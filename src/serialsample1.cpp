#include "commonsft.hpp"
#include "serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include <fstream>

//////Write buffer size/////
#define BUFFER_SIZE_W 16
//////Read buffer size/////
#define BUFFER_SIZE_R 28

/////Serial Port/////
#define COMPORT "/dev/ttyUSB0"
#define BAUDRATE B921600

/////Interval of callback(==sampling time)/////
#define INTERVAL_CALLBACK 2ms

/////Number of reading data///// 
#define NUMBER_READ 1000

/////Data to be output to console/////
//#define OUTINFO_WRITEDATA
//#define OUTINFO_READDATA
//#define OUTINFO_DIGIT
#define OUTINFO_NEWTON

/////Data to be output to textfile/////
//#define OUTFILE_DIGIT
#define OUTFILE_NEWTON

using namespace std;
using namespace std::chrono;

class SerialSample1 : public rclcpp::Node
{
	public: 
    	SerialSample1(const std::string& port, unsigned int baudrate) : Node("serialsample1")
		{
			/////Set IPAddress & Port/////
			if(sft0.init(port, baudrate) == false){
				RCLCPP_ERROR(this->get_logger(), "Error in initializing communication");
				exit(1);
			}

		    /////Set latency to low/////
		    /////If you want to go faster, need to set low_latency
    		string command = "setserial " + string(COMPORT) + " low_latency";
    		system(command.c_str());

			/////Initialize SFT/////
			InitSFT(this);

			/////Open textfile to write data/////
			//writing_file.open(filename, std::ios::app);		//append
			writing_file.open(filename, std::ios::out);			//overwrite

		    cout << "Start continuous sending of R command" << endl;
    		getchar();

        	/////Set function to be called back by timer/////
        	timer_ = this->create_wall_timer(INTERVAL_CALLBACK,
						std::bind(&SerialSample1::timer_callback, this));
    	}

		~SerialSample1(){
			writing_file.close();
		}

	private:
    	rclcpp::TimerBase::SharedPtr timer_; 
		int count_;		//counter
		bool flgCallbackRun = false;
    	serial sft0; 
		uint16_t uiFM[7];	//Force & Momnet(digit) 0-2^15 + index
		double dFM[6];		//Force & Moment(N,Nm)
		double dig2FM[6];	//coefficients to convert from digit to Force(N) and Moment(Nm)
							//index:0 Fx, 1 Fy, 2 Fz, 3 Mx, 4 My, 5Mz

		std::ofstream writing_file;
		std::string filename = "output_serialsample1.txt";

		///////////////////////////////////////////////////////////////////////
		//// For receive force data with R-command periodically from SFT sensor
		//////////////////////////////////////////////////////////////////////
		void timer_callback(){
			/////Delay the next timer execution if a callback is running/////
			if(flgCallbackRun){return;}
			flgCallbackRun = true;
				//std::this_thread::sleep_for(milliseconds(2000));	//extend callback for test

    		uint8_t wBuf[BUFFER_SIZE_W];
    		uint8_t rBuf[BUFFER_SIZE_R];

			//////Write data/////
			wBuf[0] = 'R';
			sft0.write(wBuf, 1);
			//sft0.write(wBuf, sizeof(uint8_t)* BUFFER_SIZE_W);

			/////DISPLAY write data/////
#ifdef OUTINFO_WRITEDATA
			RCLCPP_INFO(this->get_logger(), "send: %d", wBuf[0]);
#endif

			/////Read data/////
	        ssize_t readLenTotal = 0;
			while(readLenTotal < 27){
				int readLen = sft0.read(rBuf + readLenTotal, 27 - readLenTotal);
            	if(readLen <= 0){break;}
            	readLenTotal += readLen;
        	}
			if(readLenTotal == 27){
				if(rBuf[25] == '\r' && rBuf[26] == '\n'){
					/////DISPLAY read data/////
#ifdef OUTINFO_READDATA
					RCLCPP_INFO(this->get_logger(), 
					"recieve: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
					rBuf[0], rBuf[1], rBuf[2], rBuf[3], rBuf[4], rBuf[5],
					rBuf[6], rBuf[7], rBuf[8], rBuf[9], rBuf[10], rBuf[11],
					rBuf[12], rBuf[13], rBuf[14], rBuf[15], rBuf[16],rBuf[17],
					rBuf[18], rBuf[19], rBuf[20], rBuf[21], rBuf[22], rBuf[23],
					rBuf[24], rBuf[25], rBuf[26]);
#endif

					/////Get force & moment(digit)/////
					SftAscii2Dec(rBuf, uiFM);
					
					/////Conversion from digit to force(N) or moment(Nm)/////
					for(int i = 0; i < 6; i++){
						dFM[i] = (uiFM[i] - 16384) / dig2FM[i];
					}

					/////DISPLAY Force & Moment/////
#ifdef OUTINFO_DIGIT
					RCLCPP_INFO(this->get_logger(), 
						"index,Fx,Fy,Fz,Mx,My,Mz(digit): %d %d %d %d %d %d %d",
						uiFM[6], uiFM[0], uiFM[1], uiFM[2], uiFM[3], uiFM[4], uiFM[5]);
#endif
#ifdef OUTINFO_NEWTON
					RCLCPP_INFO(this->get_logger(), 
						"index,Fx,Fy,Fz,Mx,My,Mz(N,Nm): %d %f %f %f %f %f %f",
						uiFM[6], dFM[0], dFM[1], dFM[2], dFM[3], dFM[4], dFM[5]);
#endif
					/////Output Force & Moment to textfile/////
#ifdef OUTFILE_DIGIT
					writing_file << uiFM[6] << "\t"<< uiFM[0] << "\t"<< uiFM[1]
							<< "\t"<< uiFM[2] << "\t"<< uiFM[3] << "\t"<< uiFM[4]
							<< "\t"<< uiFM[5] << std::endl;
#endif
#ifdef OUTFILE_NEWTON
					writing_file << uiFM[6] << "\t"<< dFM[0] << "\t"<< dFM[1]
							<< "\t"<< dFM[2] << "\t"<< dFM[3] << "\t"<< dFM[4]
							<< "\t"<< dFM[5] << std::endl;
#endif
					/////Counting read data/////
					count_++;
					if(count_ >= NUMBER_READ){
						RCLCPP_INFO(this->get_logger(), "Finish");
						timer_->cancel();
						rclcpp::shutdown();	//escape spin()
					}
				}
				else{
					errorRead(this, wBuf[0], "Receiving position shifted");
					/////describe error handling below/////
					sft0.clearReadBuf();	
				}
			}
			else{
				errorRead(this, wBuf[0]);
				/////describe error handling below/////
				sft0.clearReadBuf();	
			}
			flgCallbackRun = false;
		}

		///////////////////////////////////////////////////////////////////////
		//// Initialize SFT(Send cmd:V,U,L#,P,O#)
		//////////////////////////////////////////////////////////////////////
		void InitSFT(SerialSample1* node){
			std::string writeData;
			std::string readData;

			/////Get ModelName/////
			writeData = "V";
			sft0.write(writeData);
				std::this_thread::sleep_for(milliseconds(1));	
			RCLCPP_INFO(node->get_logger(),"send %s", writeData.c_str());
			readData = sft0.read();
			if(readData.length() > 0){
				RCLCPP_INFO(node->get_logger(), "ModelName: %s", readData.c_str());
			}
			else{
				errorRead(node,writeData,"Error in communication with the seosor");
				exit(1);
			}
			/////Get coefficients to convert from digit to Newton(N) or Moment(Nm)/////
			writeData = "U";
			sft0.write(writeData);
				std::this_thread::sleep_for(milliseconds(1));	
			RCLCPP_INFO(node->get_logger(),"send %s", writeData.c_str());
			readData = sft0.read();
			if(readData.length() > 0){
				std::istringstream iss(readData);
				std::string word;
				int index = 0;
				while(iss >> word && index < 6){
					dig2FM[index++] = std::stod(word);	//string to double			
				}		
				RCLCPP_INFO(node->get_logger(), "dig2FM : %f %f %f %f %f %f",
					dig2FM[0], dig2FM[1], dig2FM[2], dig2FM[3], dig2FM[4], dig2FM[5]);
			}
			else{ errorRead(node,writeData); }

			/////Set LPF Filter/////
			writeData = "L1";
			sft0.write(writeData);
				std::this_thread::sleep_for(milliseconds(1));	

			/////Send P for Check LPF number/////
			writeData = "P";
			sft0.write(writeData);
				std::this_thread::sleep_for(milliseconds(1));	
			RCLCPP_INFO(node->get_logger(),"send %s", writeData.c_str());
			readData = sft0.read();
			if(readData.length() > 0){
				RCLCPP_INFO(node->get_logger(), "P-value: %s", readData.c_str());
			}
			else{ errorRead(node,writeData); }

			/////Zero Reset/////
			std::this_thread::sleep_for(milliseconds(1000));    //need wait time after setting LPF
			writeData = "O1";
			sft0.write(writeData);	
				std::this_thread::sleep_for(milliseconds(100 + 5));	//need wait time for  # * 100ms

		}

		void errorRead(SerialSample1* node, std::string& writeData,
						const std::string& message = "Failed to receive data"){
				RCLCPP_ERROR(node->get_logger(), "%s:%s", writeData.c_str(), message.c_str());
		}

		void errorRead(SerialSample1* node, int8_t writeData,
						const std::string& message = "Failed to receive data"){
				RCLCPP_ERROR(node->get_logger(), "%d:%s", writeData, message.c_str());
		}

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<SerialSample1>(COMPORT, BAUDRATE);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

