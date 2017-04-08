/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "voice_system/TTSService.h"

using namespace std;
static const char* filename = "/tmp/voice.wav";
static ros::Publisher pub_play;

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	//printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

int TextToWav(const char* text, const char* filename)
{
	int         ret                  = MSP_SUCCESS; // 58d77a1a  
	const char* login_params         = "appid = 58d87002, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/
	const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		goto exit ;//登录失败，退出登录
	}
	/* 文本合成 */
	ROS_INFO("Gen...");
	ret = text_to_speech(text, filename, session_begin_params);
	if (MSP_SUCCESS != ret)
	{
		printf("text_to_speech failed, error code: %d.\n", ret);
	}
	ROS_INFO("Finish");

exit:
	MSPLogout(); //退出登录

	return 0;
}

void playWav()
{
	// make sure the mic is umte first use amixer -c1 controls to list the controls
  //system("amixer -c 1 cset numid=3,iface=MIXER,name='Headset Capture Switch' off");
	system("amixer -c 1 cset numid=7,iface=MIXER,name='Mic Capture Switch' off");
	ROS_INFO("Start play...");
	system("play /tmp/voice.wav");
	ROS_INFO("End play...");
	system("amixer -c 1 cset numid=7,iface=MIXER,name='Mic Capture Switch' on");
}

static void ttsCallback(const std_msgs::String::ConstPtr& msg)
{
	std_msgs::Int32 msg_play;

	ROS_INFO("+%s", __func__);
	printf("%s [%s]\n", __func__, msg->data.c_str());
	//std::cout<<"Get topic text: "<< msg->data.c_str() << endl; 

	TextToWav(msg->data.c_str(), filename);

	//msg_play.data = 1;
	//ROS_INFO("%s pub 1", __func__);
	//pub_play.publish(msg_play);
	playWav();
	sleep(1);
	msg_play.data = 0;
	ROS_INFO("%s pub 0", __func__);
	pub_play.publish(msg_play);	
	
	ROS_INFO("-%s", __func__);
}

static bool ttsService(voice_system::TTSService::Request &req, voice_system::TTSService::Response &res)
{
	res.result = true;

	printf("+%s play [%s]", __func__, req.target.c_str());

	TextToWav(req.target.c_str(), filename);
	playWav();
	sleep(1);

#if 0
	printf("%s [%s]\n", __func__, msg->data.c_str());
	//std::cout<<"Get topic text: "<< msg->data.c_str() << endl; 

	TextToWav(msg->data.c_str(), filename);

	//msg_play.data = 1;
	//ROS_INFO("%s pub 1", __func__);
	//pub_play.publish(msg_play);
	playWav();
	sleep(1);
	msg_play.data = 0;
	ROS_INFO("%s pub 0", __func__);
	pub_play.publish(msg_play);	
#endif	
	ROS_INFO("-%s", __func__);
	return true;
}


int main(int argc, char* argv[])
{
	const char* start = "在线语音合成模块启动";

	ros::init(argc, argv, "xf_tts_node");

	ros::NodeHandle n;

	ros::ServiceServer tts_service = n.advertiseService("tts_service", ttsService);

	TextToWav(start, filename);
	playWav();

	// set 1 if there is playing
	pub_play = n.advertise<std_msgs::Int32>("/voice/xf_tts_playing", 50);

	// get msg published by tuling/ASR, and play it back
	ros::Subscriber sub = n.subscribe("/voice/xf_tts_topic", 50, ttsCallback);
	// rostopic pub  /voice/xf_tts_topic std_msgs/String "start ..."

	ros::spin();

	return 0;
}

