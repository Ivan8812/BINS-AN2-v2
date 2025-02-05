#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <tuple>

class AHRS
{
public:

	template<int R, int C>
	using matrix_t = Eigen::Matrix<float,R,C>;

	template<int N>
	using vector_t = Eigen::Vector<float, N>;

	using vector3_t = Eigen::Vector<float,3>;

	using quaternion_t = Eigen::Quaternion<float>;

	AHRS(float sample_rate = 100) : Fs(sample_rate) {reset();}

	//--------------------------------------------------------------------------
	// текущий кватернион ориентации
	quaternion_t current_quat() { return orientation; }
	//--------------------------------------------------------------------------
	// представление текущей ориентации в виде углов эйлера
	vector3_t current_euler() { return quat2euler(orientation); }
	//--------------------------------------------------------------------------
	// текущее смещение гиро
	vector3_t gyr_offset() { return gyr_offs; }
	//--------------------------------------------------------------------------
	// текущее кажущееся ускорение
	vector3_t free_acc() { return (LinAccDecay > 0) ? lin_acc / LinAccDecay : vector3_t({0,0,0}); }
	//--------------------------------------------------------------------------
	// режим выставки
	void set_no_rotation(float t)
	{
		if(roundf(t*Fs) > no_rot_i)
		{
			no_rot_i = roundf(t*Fs);
			gyr_i = 0;
			gyr_sum = {0,0,0};
			acc_sum = mag_sum = 0;
		}
	}
	//--------------------------------------------------------------------------
	void reset()
	{
		orientation = quaternion_t( 1, 0, 0, 0 );
		gyr_offs = g_gyr = m_gyr = m = lin_acc = mag_dis = gyr_sum = {0, 0, 0};
		acc_norm = mag_norm = 1;
		acc_sum = mag_sum = 0;
		
		float f = Fs/(float)DecN;
		float g_noise = AccNoise + LinAccNoise + (GyrDriftNoise + GyrNoise)/f/f;
		float m_noise = MagNoise + MagDisNoise + (GyrDriftNoise + GyrNoise)/f/f;
		measure_cov = vector_t<6>({ g_noise, g_noise, g_noise, m_noise, m_noise, m_noise }).asDiagonal();

		state_cov = vector_t<12>({ OrientNoise, OrientNoise, OrientNoise,
		                           GyrNoise, GyrNoise, GyrNoise,
		                           LinAccNoise, LinAccNoise, LinAccNoise,
		                           MagDisNoise, MagDisNoise, MagDisNoise }).asDiagonal();
		
		data_valid = init_complete = false;
		dec_i = gyr_i = no_rot_i = 0;
	}
	//--------------------------------------------------------------------------
	// шаг обновления ориентации. ф-ия должна вызываться при поступлении свежих инерциальных данных
	// с частотой Fs, которая задается при создании экземпляра класса
	void step(const vector3_t& acc, const vector3_t& gyr, const vector3_t& mag)
	{
		using Eigen::seq;
		using Eigen::all;

			// на первом шаге априорная оценка ориентации берется из измерения аксов и магнитометра,
			// на последующих шагах априорная оценка ориентации считается на основе данных гиро
		if (!init_complete)
		{
			std::tie(orientation, m) = ecompass(acc, mag);
			init_complete = true;
		}
		else
			orientation *= rotvec2quat((gyr - gyr_offs)/Fs);

			// во время выставки смещения гиросов измеряется непосредственно из сигналов
		if(no_rot_i)
		{
			gyr_i++;
			gyr_sum += gyr;
			acc_sum += acc.norm();
			mag_sum += mag.norm();
			gyr_offs = gyr_sum/((float)gyr_i);
			acc_norm = acc_sum/((float)gyr_i);
			mag_norm = mag_sum/((float)gyr_i);			
			no_rot_i--;
			return;
		}

			// калмановскую оценку ошибок можно проводить на каждом DecN шаге, если есть дефецит
			// вычислительных ресурсов, это увеличит дисперсию выходных данных, мат.ожидание ошибок оценивается так же
		if (++dec_i < DecN)
			return;
		dec_i = 0;

			// оценка вектора g и вектора магнитного поля в проекции на подвижную СК
		matrix_t<3,3> rotmat = orientation.toRotationMatrix().transpose();		
		g_gyr = rotmat(all, 2);
		m_gyr = rotmat*m;

			// вектор ошибки - разница между измерениями векторов g,m и их оценкой
		vector_t<6> err;
		err(seq(0, 2)) = acc/acc_norm + lin_acc - g_gyr;
		err(seq(3, 5)) = mag/mag_norm - mag_dis - m_gyr;

			// уравнения фильтра калмана. классический фильтр калмана с вектором состояния, составленным из ошибок ориентации,
			// гироскопов, аксов и магнитометра - [ox, oy, oz, gx, gy, gz, ax, ay, az, mx, my, mz].
			// моделируется уравнения ошибок с гауссовским шумом и нулевым мат.ожиданием, поэтому априорная оценка
			// вектора состояния нулевая на каждом шаге, следовательно, и переходная матрица системы тоже нулевая.
		state_cov = calc_noise_cov();
		auto meas_matr = calc_measure_matr();
		matrix_t<12, 6> gain = state_cov*meas_matr.transpose()*((meas_matr*state_cov*meas_matr.transpose() + measure_cov).inverse());
		vector_t<12> x = gain*err;
		state_cov -= gain*meas_matr*state_cov;

		// коррекция
		orientation *= rotvec2quat(-x(seq(0, 2)));
		orientation.normalize();
		gyr_offs -= 0.01f*x(seq(3, 5));
		lin_acc = lin_acc*LinAccDecay - x(seq(6, 8));
		mag_dis = mag_dis*MagDisDecay - x(seq(9, 11));
		m -= orientation.toRotationMatrix()*x(seq(9, 11));
		auto incl = atan2f(m(2), m(0));
		m = { cosf(incl), 0.0, sinf(incl) };
	}
	//--------------------------------------------------------------------------
	static std::tuple<quaternion_t, vector3_t> ecompass(const vector3_t& acc, const vector3_t& mag)
	{
		// нормализованные вектора rx,ry,rz образуют матрицу поворота
		vector3_t rz(acc.normalized());
		vector3_t ry(rz.cross(mag).normalized());
		vector3_t rx(ry.cross(rz).normalized());

		float qw = sqrtf(1 + rx(0) + ry(1) + rz(2))/2;
		float qx = (rz(1) - ry(2))/4/qw;
		float qy = (rx(2) - rz(0))/4/qw;
		float qz = (ry(0) - rx(1))/4/qw;
		quaternion_t q(qw, qx, qy, qz);

		float incl = asinf(acc.normalized().dot(mag.normalized()));
		vector3_t m({ cosf(incl), 0, sinf(incl) });

		return { q,m };
	}
	//--------------------------------------------------------------------------
	static inline quaternion_t rotvec2quat(const vector3_t& v)
	{
		float n = v.norm();
		if (n == 0)
			return quaternion_t( 1, 0, 0, 0 );
		return quaternion_t(cosf(n/2), v[0]/n*sinf(n/2), v[1]/n*sinf(n/2), v[2]/n*sinf(n/2));
	}
	//--------------------------------------------------------------------------
	static vector3_t quat2euler(const quaternion_t &q)
	{
		float qw = q.w();
		float qx = q.x();
		float qy = q.y();
		float qz = q.z();

		return vector3_t({ atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)),
						  -pi / 2 + 2 * atan2f(sqrtf(1 + 2 * (qw * qy - qx * qz)), sqrtf(1 - 2 * (qw * qy - qx * qz))),
						   atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) }) * 180 / pi;
	}
	//--------------------------------------------------------------------------

private:
	static constexpr float OrientNoise = 1;

	static constexpr float AccNoise = 2e-6f;
	static constexpr float LinAccNoise = 2e-2f;
	static constexpr float LinAccDecay = 0.5f;

	static constexpr float GyrNoise = 1e-5f;
	static constexpr float GyrDriftNoise = 1e-14f;

	static constexpr float MagNoise = 2e-6f;
	static constexpr float MagDisNoise = 2e-2f;
	static constexpr float MagDisDecay = 0.5f;

	static constexpr float pi = 3.14159265358979323846f;

	static const uint32_t DecN = 1;

	const float Fs;
	bool data_valid = false;
	bool init_complete = false;
	quaternion_t orientation;
	vector3_t gyr_offs, g_gyr, m_gyr, m, lin_acc, mag_dis, gyr_sum;
	matrix_t<12, 12> state_cov;
	matrix_t<6, 6> measure_cov;
	float acc_norm = 1, acc_sum = 0, mag_norm = 1, mag_sum = 0;
	uint32_t dec_i = 0, gyr_i = 0, no_rot_i = 0;
	
	//--------------------------------------------------------------------------
	// обновление ковариационной матрицы вектора состояния для фильтра калмана
	inline Eigen::Matrix<float, 12, 12> calc_noise_cov()
	{
		matrix_t<12, 12> cov(Eigen::Array<float,12,12>::Zero());
		float f = Fs/(float)DecN;
		cov(0, 0) = state_cov(0, 0)*0.999999f + (state_cov(3, 3) + GyrDriftNoise + GyrNoise)/f/f;
		cov(1, 1) = state_cov(1, 1)*0.999999f + (state_cov(4, 4) + GyrDriftNoise + GyrNoise)/f/f;
		cov(2, 2) = state_cov(2, 2)*0.999999f + (state_cov(5, 5) + GyrDriftNoise + GyrNoise)/f/f;
		cov(3, 3) = state_cov(3, 3)*0.999999f + GyrDriftNoise;
		cov(0, 3) =-cov(3, 3)/f;
		cov(3, 0) =-cov(3, 3)/f;
		cov(4, 4) = state_cov(4, 4)*0.999999f + GyrDriftNoise;
		cov(1, 4) =-cov(4, 4)/f;
		cov(4, 1) =-cov(4, 4)/f;
		cov(5, 5) = state_cov(5, 5)*0.999999f + GyrDriftNoise;
		cov(2, 5) =-cov(5, 5)/f;
		cov(5, 2) =-cov(5, 5)/f;
		cov(6, 6) = state_cov(6, 6)*LinAccDecay*LinAccDecay + LinAccNoise;
		cov(7, 7) = state_cov(7, 7)*LinAccDecay*LinAccDecay + LinAccNoise;
		cov(8, 8) = state_cov(8, 8)*LinAccDecay*LinAccDecay + LinAccNoise;
		cov(9, 9)   = state_cov( 9, 9 )*MagDisDecay*MagDisDecay + MagDisNoise;
		cov(10, 10) = state_cov(10, 10)*MagDisDecay*MagDisDecay + MagDisNoise;
		cov(11, 11) = state_cov(11, 11)*MagDisDecay*MagDisDecay + MagDisNoise;
		return cov;
	}
	//--------------------------------------------------------------------------
	// формирование ковариационной матрицы наблюдения для фильтра калмана
	inline matrix_t<6, 12> calc_measure_matr()
	{
		float gx = g_gyr(0), gy = g_gyr(1), gz = g_gyr(2);
		float mx = m_gyr(0), my = m_gyr(1), mz = m_gyr(2);
		float f = Fs/(float)DecN;
		return matrix_t<6, 12>
		({ 
			{  0,  gz, -gy,     0, -gz/f,  gy/f,  1,  0,  0,  0,  0,  0},
			{-gz,   0,  gx,  gz/f,     0, -gx/f,  0,  1,  0,  0,  0,  0},
			{ gy, -gx,   0, -gy/f,  gx/f,     0,  0,  0,  1,  0,  0,  0},
			{  0,  mz, -my,     0, -mz/f,  my/f,  0,  0,  0, -1,  0,  0},
			{-mz,   0,  mx,  mz/f,     0, -mx/f,  0,  0,  0,  0, -1,  0},
			{ my, -mx,   0, -my/f,  mx/f,     0,  0,  0,  0,  0,  0, -1}
		});
	}
	//--------------------------------------------------------------------------
};
