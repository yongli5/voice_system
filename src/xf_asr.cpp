/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>;
#include <std_msgs/Int32.h>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

#define ASRSTART        1

using namespace std;
string result;
bool recflag = true;
int flag = 0;

static void show_result(char *str, char is_over)
{
	printf("\rResult: [ %s ]", str);
	if(is_over)
		putchar('\n');

	string s(str);
	result = s;
	flag = 1;
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT) {
           printf("\nSpeaking done \n");
	
	   recflag = false;
	}
	else
	   printf("\nRecognizer error %d\n", reason);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(recflag) {
//	   printf("recflag %d\n", recflag);
           sleep(1);	
	}
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

void asrProcess()
{
        int ret = MSP_SUCCESS;
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 58d77a1a, work_dir = .";

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}

	printf("Demo recognizing the speech from microphone\n");
	printf("Speak in 15 seconds\n");

	demo_mic(session_begin_params);

	printf("15 sec passed\n");
exit:
	MSPLogout(); // Logout...


}

void asrCallback(const std_msgs::Int32::ConstPtr& msg)
{
     std::cout<<"Now revoke record asrCallback .. " << endl; 
     
     if (msg->data == ASRSTART) {
        asrProcess(); 
     
     }
}

int main(int argc, char* argv[])
{

        ros::init(argc, argv, "xf_asr_node");

	ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe("/voice/xf_asr_topic", 50, asrCallback);

	ros::Publisher pub = n.advertise<std_msgs::String>("/voice/tuling_nlu_topic", 50);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		if (flag)
		{
			std_msgs::String msg;	
			msg.data = result;
			pub.publish(msg);
			recflag = true;
			flag =0;
		}
		loop_rate.sleep();
                ros::spinOnce();
	 }

	return 0;
}
