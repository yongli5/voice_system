/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

#define ASRSTART        1
#define SYS_UNAUTH		0
#define SYS_AUTH		1

using namespace std;
//static string result;
static bool sys_locked = true;
static bool speech_end = false;
static bool playing = false;
static int asr_flag = 0;
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

struct st_command {
	char command[255];
	unsigned int code;
};

static struct st_command voice_commands[255];

static char *test_commands[] = {"start", "stop"};

static void show_result(char *str, char is_over)
{
	int i = 0;
	int len = strlen(str);

	printf("+%s [%s]\n", __func__, str);

	for (i = 0; i < len; i++) {
		//printf("%x:", str[i]);
	}
	printf("\n");

	ROS_INFO("+%s [%s]", __func__, str);
	//printf("\rResult: [%s]", str);
	if(is_over)
		putchar('\n');

	//string s(str);
	//result = s;
	asr_flag = 1;
}

static void on_result(const char *result, char is_last)
{
	printf("+%s [%s]\n", __func__, result);
	ROS_INFO("+%s result=%p is_last=%d", __func__, result, is_last);
	
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
		if (is_last) {
			show_result(g_result, is_last);
		}
	}
}

static void on_speech_begin()
{
	ROS_INFO("+%s %p", __func__, g_result);
	if (g_result) {
		free(g_result);
		g_result = NULL;
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	speech_end = false;
	ROS_INFO("-%s %p\n", __func__, g_result);
}

static void on_speech_end(int reason)
{
	ROS_INFO("+%s %d\n", __func__, reason);
	if (reason == END_REASON_VAD_DETECT) {
		ROS_INFO("Speaking done \n");
	}
	else {
		ROS_ERROR("Recognizer error: %d\n", reason);
	}
	speech_end = true;
	
	ROS_INFO("-%s %d\n", __func__, reason);
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

	ROS_INFO("+%s", __func__);

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		ROS_ERROR("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		ROS_ERROR("start listen failed %d\n", errcode);
	}
	/* wait for recording end*/
	while(!speech_end) {
		//printf("recflag %d\n", recflag);
		sleep(1);
	}
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		ROS_ERROR("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
	ROS_INFO("-%s", __func__);
}

static void asrProcess()
#if 0
{
	ROS_INFO("+%s fake\n", __func__);
	if (g_result) {
		free(g_result);
		g_result = NULL;
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	strcpy(g_result, "启动。");

	speech_end = false;
	speech_end = true;
	sleep(5);
}
#else
{
	int ret = MSP_SUCCESS;
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 58d87002, work_dir = .";

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	ROS_INFO("+%s", __func__);

	if (g_result) {
		free(g_result);
		g_result = NULL;
	}

	// If there is anything is playing, skip the asr

	ROS_INFO("playing=%d", playing);
	if (playing) {
		ROS_INFO("playing skip ...");
		return;
	}

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}

	ROS_INFO("Recognizing the speech from microphone");

	demo_mic(session_begin_params);

exit:
	ROS_INFO("-%s logout", __func__);
	MSPLogout(); // Logout...

}
#endif

#if 0
static void asrCallback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("+%s %d", __func__, msg->data);
     //std::cout<<"Now revoke record asrCallback .. " << endl; 
     
     if (msg->data == ASRSTART) {
        asrProcess(); 
     }
	 ROS_INFO("-%s", __func__);
}
#endif

static void faceCallback(const std_msgs::Int8::ConstPtr& msg)
{
	ROS_INFO("+%s %d", __func__, msg->data);

	if (msg->data == SYS_AUTH) {
		sys_locked = false;
	} else {
		sys_locked = true;
	}

	ROS_INFO("-%s", __func__);
}

static void ttsplayCallback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("+%s %d", __func__, msg->data);
	if (msg->data == 1) {
		playing = true;
	} else {
		playing = false;
	}
}

// get voice txt- command list
static int read_config() {
    char line[255];
    char command[255];
    int code;
    FILE *f;
    int ret;
    int i, len;

    memset(&voice_commands, 0, sizeof(voice_commands));

    f = fopen("/etc/commands.txt", "re");

    if (!f) {
        printf("file open fail %d\n", errno);
    }

    //line = NULL;
    len = 0;
    while (fscanf(f, "%s[^\n]\n", (char *)&line) != EOF) {
        printf("line=[%s]\n", line);

        if (sscanf(line, "%[^-]-%d", (char *)&command, &code) != 2) {
        	printf("[%s] scan error! command=[%s] code=[%d]\n", line, command, code);
        	continue;
        }
        
        printf("command=[%s]%zu code=%d\n", command, strlen(command), code);
        strcpy(voice_commands[len].command, command);
        voice_commands[len].code = code;
        len++;
        
        //free(line);
        //line = NULL;
    }

    //for (i = 0; i < len; i++) 
    i = 0;
    while (strlen(voice_commands[i].command) != 0)
    {
        printf("{[%s]-%d}\n", voice_commands[i].command, voice_commands[i].code);
        i++;
    }

    fclose(f);    
  return 0;	
	
}

static int search_command(const char *command) {
	int i = 0;
	int code = -1;
	char *found = NULL;
	
	printf("+%s [%s]\n", __func__, command);

	i = 0;
	code = -1;
	while (0 != strlen(voice_commands[i].command)) {
		printf("%d=[%s]\n", i, voice_commands[i].command);
		found = strstr((char *)command, voice_commands[i].command);
		if (found) {
			printf("%s %d find [%s]-[%s] code=%d\n", __func__, i, command, voice_commands[i].command, voice_commands[i].code);
			code = voice_commands[i].code;
			break;
		}
		i++;
	}

	printf("-%s [%s] code=%d\n", __func__, command, code);
	return code;
}

int main(int argc, char* argv[])
{
	char tts_content[255];
	int code = 0;
	//std::cout << "asr start ..." << endl; 
    ros::init(argc, argv, "xf_asr_node");

	ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/voice/xf_asr_topic", 50, asrCallback);
    ros::Subscriber sub = n.subscribe("/face/auth", 50, faceCallback);

    ros::Subscriber sub_ttsplay = n.subscribe("/voice/xf_tts_playing", 50, ttsplayCallback);
	
	// publish to tts, play back the received voice
	ros::Publisher pub_tts = n.advertise<std_msgs::String>("/voice/xf_tts_topic", 10);

	ros::Publisher pub_text = n.advertise<std_msgs::String>("/voice/tuling_nlu_topic", 50);
	ros::Publisher pub_cmd = n.advertise<std_msgs::Int32>("/voice/cmd_topic", 50);

	ros::Rate loop_rate(10);

	ROS_INFO("start listen ...");

	read_config();
	//std::cout << "start listen ..." << endl;
	while (ros::ok())
	{
		// listen .. 
		asrProcess();

		//if (asr_flag)
		{
			std_msgs::String msg;
			std_msgs::Int32 cmd_msg;

			if ((g_result == NULL) || (0 == strlen(g_result))) {
				ROS_INFO("no voice detected");
				continue;
			}

			// play back the received voice
			std_msgs::String msg_tts;
			// if system is locked, ignore voice input too
			if (0) //(sys_locked) 
			{
				ROS_INFO("sys_locked, skip voice!");

				memset(tts_content, 0, sizeof(tts_content));
				sprintf(tts_content, "收到 %s", g_result);
				strcat(tts_content, "系统锁定 被忽略!");
				msg_tts.data = tts_content;
				pub_tts.publish(msg_tts);
				sleep(6);
				continue;
			}

			msg.data = g_result;

			code = search_command(g_result);
			if (code >= 0) { // the voice is a special command
				cmd_msg.data = code;
				ROS_INFO("Publish command [%d]", code);

				memset(tts_content, 0, sizeof(tts_content));
				sprintf(tts_content, "执行命令 %s", g_result);
				msg_tts.data = tts_content;
				pub_tts.publish(msg_tts);
				pub_cmd.publish(cmd_msg);
				sleep(8);
			} else { // send to tuling
				printf("publish [%s]\n", g_result);
				ROS_INFO("Publish [%s]", msg.data.c_str());

				memset(tts_content, 0, sizeof(tts_content));
				sprintf(tts_content, "未识别 %s 请机器人帮忙", g_result);
				msg_tts.data = tts_content;
				pub_tts.publish(msg_tts);
				//playing = true;
				pub_text.publish(msg);
				sleep(8);
			}

			//speech_end = true;
			//asr_flag =0;
		}
		loop_rate.sleep();
        ros::spinOnce();
	 }

	return 0;
}
