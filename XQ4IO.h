#pragma once
#include <asio.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
using namespace std;

#ifndef tsns
#define tsns chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsus chrono::time_point_cast<chrono::microseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsms chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsse chrono::time_point_cast<chrono::seconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsmi chrono::time_point_cast<chrono::minutes>(chrono::system_clock::now()).time_since_epoch().count()
#define tsho chrono::time_point_cast<chrono::hours>(chrono::system_clock::now()).time_since_epoch().count()
#endif

template<typename Object, int nbuf = 9, int nchn = 9, int ndim = 6> class CirArr
{
public:
	static CirArr& GetMe(int dim = 0) { static CirArr us[ndim]; return us[dim]; }

protected:
	bool state = false;
	Object objects[nbuf];
	int64 readPos[nchn] = { 0 };
	int64 writePos = 0;

public:
	bool getState() { return state; }
	int getBuf() { return nbuf; }
	int getChn() { return nchn; }
	int getDim() { return ndim; }

public:
	virtual bool init()
	{
		if (state == true) { spdlog::info("zero operation"); return true; }
		memset(objects, 0, sizeof(objects));
		memset(readPos, 0, sizeof(readPos));
		memset(&writePos, 0, sizeof(writePos));
		state = true;
		return true;
	}
	virtual bool deinit()
	{
		if (state == false) { spdlog::info("zero operation"); return true; }
		memset(objects, 0, sizeof(objects));
		memset(readPos, 0, sizeof(readPos));
		memset(&writePos, 0, sizeof(writePos));
		state = false;
		return true;
	}

public:
	bool getLatest(Object** object, int chnId, int msTimeout = 1000, int msSleep = 2)
	{
		if (state != true) { spdlog::error("wrong state"); return false; }
		for (int64 t0 = tsms; tsms - t0 < msTimeout;)
		{
			int64 availablePos = writePos;
			if (availablePos > readPos[chnId])
			{
				int64 relativePos = availablePos % nbuf;
				*object = objects + relativePos;
				readPos[chnId] = availablePos;
				return true;
			}
			this_thread::sleep_for(chrono::milliseconds(msSleep));
		}
		return false;
	}
	int64 lockWritten(Object** object)
	{
		if (state != true) { spdlog::error("wrong state"); return -1; }
		int64 absolutePos = writePos;
		int64 relativePos = ++absolutePos % nbuf;
		*object = objects + relativePos;
		return absolutePos;
	}
	int64 unlockWritten(int64 absolutePos)
	{
		if (state != true) { spdlog::error("wrong state"); return -1; }
		return (writePos = absolutePos);
	}
};

class XQ4IO
{
public:
	struct XQ4Frame
	{
		int status;//小车状态: 0未初始化, 1正常, -1表示异常
		float power;//电源电压: 9~13V
		float theta;//方位角:0~360Deg
		int encoder_ppr;//车轮一转对应的编码器个数
		int encoder_delta_r;//右轮编码器增量: 单位个
		int encoder_delta_l;//左轮编码器增量: 单位个
		int encoder_delta_car;//两车轮中心位移: 单位个
		int omga_r;//右轮转速: 个每秒
		int omga_l;//左轮转速: 个每秒
		float distance1;//第一个超声模块距离值: 单位cm
		float distance2;//第二个超声模块距离值: 单位cm
		float distance3;//第三个超声模块距离值: 单位cm
		float distance4;//第四个超声模块距离值: 单位cm
		float imudata[9];//mpu9250 9轴数据
		uint timestamp;//时间戳
		string print(string savePath = "")
		{
			string str;
			str += fmt::format("status: {}\n", status);
			str += fmt::format("power: {}\n", power);
			str += fmt::format("theta: {}\n", theta);
			str += fmt::format("encoder_ppr: {}\n", encoder_ppr);
			str += fmt::format("encoder_delta_r: {}\n", encoder_delta_r);
			str += fmt::format("encoder_delta_l: {}\n", encoder_delta_l);
			str += fmt::format("encoder_delta_car: {}\n", encoder_delta_car);
			str += fmt::format("omga_r: {}\n", omga_r);
			str += fmt::format("omga_l: {}\n", omga_l);
			str += fmt::format("distance1: {}\n", distance1);
			str += fmt::format("distance2: {}\n", distance2);
			str += fmt::format("distance3: {}\n", distance3);
			str += fmt::format("distance4: {}\n", distance4);
			for (int k = 0; k < 9; ++k) str += fmt::format("imudata[{}]: {}\n", k, imudata[k]);
			str += fmt::format("timestamp: {}\n", timestamp);
			return str;
		}
	};

private://1.Device
	string sname;
	asio::io_service io;
	asio::serial_port sport = asio::serial_port(io);
	bool keeping = false;
	int64 msTimeout = 10;
	CirArr<XQ4Frame> cirFrames;

public://2.Config
	inline bool open(string spname)
	{
		error_code ec;
		sport.open(sname = spname, ec);
		if (ec.value() != 0) return false;
		cirFrames.init();
		using namespace asio;
		sport.set_option(serial_port::baud_rate(115200));
		sport.set_option(serial_port::character_size(8));
		sport.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
		sport.set_option(serial_port::parity(serial_port::parity::none));
		sport.set_option(serial_port::flow_control(serial_port::flow_control::none));
		std::thread(&XQ4IO::read2decode, this).detach();
		return keeping = true;
	}
	inline bool opened() { return sport.is_open(); }
	inline void close() { keeping = false; this_thread::sleep_for(1000ms); cirFrames.deinit(); sport.close(); }
	inline string name() { return sname; }

public://3.Write
	const char heads[3] = { char(0xcd), char(0xeb), char(0xd7) };
	inline void setMode(char mode/*T/R/I*/)//T=debug   R=work   I=reset
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: mode={}", mode); return; }
		char data[5] = { heads[0], heads[1], heads[2], char(0x01), mode };
		asio::write(sport, asio::buffer(data, 5));
	}
	inline void runCar(char action/*f/b/s/c/d*/, char velocity/*0~100*/)//f=move ahead   b=move back   s=brake   c=turn left   d=turn right
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action={}   velocity={}", action, int(velocity)); return; }
		char data[6] = { heads[0], heads[1], heads[2], char(0x02), action, velocity };
		asio::write(sport, asio::buffer(data, 6));
	}
	inline void runMotor(char action1/*F/B/S*/, char action2/*F/B/S*/, char velocity1/*0~100*/, char velocity2/*0~100*/)
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action1={}   action2={}   velocity1={}   velocity2={}", action1, action2, int(velocity1), int(velocity2)); return; }
		char data[13] = { heads[0], heads[1], heads[2], char(0x09), char(0x74), action1, action2, 0x53, 0x53, velocity1, velocity2, char(0x00), char(0x00) };
		asio::write(sport, asio::buffer(data, 13));
	}
	inline void runSensor(int action/*0=CalibIMU 1=EnableIR 2=DisableIR*/)
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action={}", action == 0 ? "CalibIMU" : action == 1 ? "EnableIR" : "DisableIR"); return; }
		char data[6] = { heads[0], heads[1], heads[2], action == 0 ? char(0x01) : char(0x02), action == 0 ? char(0x43) : char(0x44), action == 2 ? char(0x00) : char(0x01) };
		asio::write(sport, asio::buffer(data, action == 0 ? 5 : 6));
	}

	inline void write(char data[], int n)
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned,write failed"); return; }
		asio::write(sport, asio::buffer(data, n));
	}
	
        inline bool getStatus(XQ4Frame **frame, int chnId = 0, int msTimeout = 1000, int msSleep = 5) { return cirFrames.getLatest(frame, chnId, msTimeout, msSleep); }	
	string strDBG;

private:
	static const int headSize = 4;
	static const int itemSize = sizeof(int);
	static const int itemSizeEx = itemSize + 1;
	static const int itemCount = sizeof(XQ4IO::XQ4Frame) / itemSize;
	static const int dataSize = itemCount * itemSizeEx;
	static const int frameSize = dataSize + headSize;
	void read2decode()
	{
		int64_t frameTotal = 0;//from start 
		int64_t frameCount = 0;//in one second
		int64_t firstStamp = tsms;
		int64_t preStamp = firstStamp;
		while (keeping)
		{
			//1.ReadHead
			int workable = 0;
			for (int k = 0; k < 4; ++k)
			{
				error_code ec;
				char headFlag;
				int num = sport.read_some(asio::buffer(&headFlag, 1), ec);
				if (num != 1) spdlog::error(strDBG = fmt::format("Line{}Exception: num={}, code={}, msg={}", __LINE__, num, ec.value(), ec.message()));
				else if (headFlag == heads[k]) ++workable; //check headFlag
				else if (headFlag == dataSize) ++workable; //check dataSize
			}

			//2.ReadData
			if (workable == 4)
			{
				error_code ec;
				char dataDetail[dataSize];
				int num = asio::read(sport, asio::buffer(&dataDetail, dataSize), ec);
				if (num != dataSize) spdlog::error(strDBG = fmt::format("Line{}Exception: num={}, code={}, msg={}", __LINE__, num, ec.value(), ec.message()));
				else
				{
					XQ4IO::XQ4Frame* frame;
					int64 tsPos = cirFrames.lockWritten(&frame);
					for (int k = 0; k < itemCount; ++k) memcpy((int*)frame + k, dataDetail + itemSizeEx * k, itemSize);
					//cout << endl << frame->print() << endl;
					cirFrames.unlockWritten(tsPos);
					++frameTotal;
					++frameCount;
				}
			}

			//3.CheckFPS
			int64_t curStamp = tsms;
			int64_t diffDuration = curStamp - preStamp;
			if (diffDuration > 1000)
			{
				float fps = frameCount * 1000.f / diffDuration;
				if (fps < 49) spdlog::warn(strDBG = fmt::format("Line{}Warn: CurrentFPS={}", __LINE__, fps));
				frameCount = 0;
				preStamp = curStamp;
				if ((curStamp - firstStamp) / 1000 % 3 == 0) spdlog::info(strDBG = fmt::format("Line{}Info: AverageFPS={}", __LINE__, frameTotal * 1000.f / (curStamp - firstStamp)));
			}
		}
	}

public:
	static void XQ4Sim(string sportname)
	{
		//1.OpenPort
		error_code ec;
		asio::io_service io;
		asio::serial_port sport = asio::serial_port(io);
		sport.open(sportname, ec);
		if (ec.value() != 0) {spdlog::error("Open sport {} failed: {}", sportname, ec.message()); return; }

		//2.SetPort
		sport.set_option(asio::serial_port::baud_rate(115200));
		sport.set_option(asio::serial_port::character_size(8));
		sport.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
		sport.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
		sport.set_option(asio::serial_port::flow_control(asio::serial_port::flow_control::none));

		//3.WritePort
		asio::io_context ioc;
		asio::steady_timer timer(ioc, 1000ms);
		std::function<void(error_code ec)> SendXQStatus;
		SendXQStatus = [&](error_code ec)->void
		{
			//3.1 RunTask
			int64_t last_expiry_stamp = chrono::time_point_cast<chrono::milliseconds>(timer.expiry()).time_since_epoch().count();
			static int virvalue = -1; virvalue = virvalue + 100 > INT_MAX ? -1 : virvalue;
			char data[frameSize] = { 0 };
			data[0] = 0xcd; data[1] = 0xeb; data[2] = 0xd7; data[3] = dataSize;
			int *status = (int*)(data + 4); *status = 1;
			float* power = (float*)(data + 4 + 5 * 1); *power = 11;
			float* theta = (float*)(data + 4 + 5 * 2); *theta = ++virvalue;
			int* encoder_ppr = (int*)(data + 4 + 5 * 3); *encoder_ppr = ++virvalue;
			int* encoder_delta_r = (int*)(data + 4 + 5 * 4); *encoder_delta_r = ++virvalue;
			int* encoder_delta_l = (int*)(data + 4 + 5 * 5); *encoder_delta_l = ++virvalue;
			int* encoder_delta_car = (int*)(data + 4 + 5 * 6); *encoder_delta_car = ++virvalue;
			int* omga_r = (int*)(data + 4 + 5 * 7); *omga_r = ++virvalue;
			int* omga_l = (int*)(data + 4 + 5 * 8); *omga_l = ++virvalue;
			float* distance1 = (float*)(data + 4 + 5 * 9); *distance1 = ++virvalue;
			float* distance2 = (float*)(data + 4 + 5 * 10); *distance2 = ++virvalue;
			float* distance3 = (float*)(data + 4 + 5 * 11); *distance3 = ++virvalue;
			float* distance4 = (float*)(data + 4 + 5 * 12); *distance4 = ++virvalue;
			for(int k = 13; k < 13 + 9; ++k) { float* imu = (float*)(data + 4 + 5 * k); *imu = ++virvalue; }
			uint *timestamp = (uint*)(data + 4 + 5 * 22); *timestamp = tsse;
			asio::write(sport, asio::buffer(data, frameSize));

			//3.2 DelayTimer
			timer.expires_at(timer.expires_at() + 20ms);//50HZ
			timer.async_wait(SendXQStatus);

			//3.3 PrintTime
			int64_t next_expiry_stamp = chrono::time_point_cast<chrono::milliseconds>(timer.expiry()).time_since_epoch().count();
			int64_t current_stamp = chrono::time_point_cast<chrono::milliseconds>(chrono::steady_clock::now()).time_since_epoch().count();
			spdlog::info("Timestamp{}: TaskTimeCost={}   IdleTimeCost={}   Theta={}", *timestamp, next_expiry_stamp - current_stamp, current_stamp - last_expiry_stamp, *theta);
		};
		timer.async_wait(SendXQStatus);
		ioc.run();
	}
};