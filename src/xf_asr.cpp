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
#include <geometry_msgs/Twist.h>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

enum state_codes { entry, foo, bar, end, fail_code};
enum ret_codes { ok, fail, repeat};

#define EXIT_STATE end
#define ENTRY_STATE entry
#define FOO_STATE foo
#define BAR_STATE bar

#define CODE_OK  ok
#define CODE_FAIL fail
#define CODE_REPEAT repeat

struct transition {
    enum state_codes src_state;
    enum ret_codes   ret_code;
    enum state_codes dst_state;
};
/* transitions from end state aren't needed */
struct transition state_transitions[] = {
    {entry, ok,     foo},
    {entry, fail,   end},
    {foo,   ok,     bar},
    {foo,   fail,   end},
    {foo,   repeat, foo},
    {bar,   ok,     end},
    {bar,   fail,   end},
    {bar,   repeat, foo}};

typedef enum ret_codes FUNC(void);

static enum ret_codes entry_state(void)
{
    printf("+%s\n", __func__);
    return CODE_OK;
}
static enum ret_codes foo_state(void)
{
    printf("+%s\n", __func__);
    return CODE_OK;
}
static enum ret_codes bar_state(void)
{
    printf("+%s\n", __func__);
    return CODE_OK;
}
static enum ret_codes exit_state(void)
{
    printf("+%s\n", __func__);
}

/* array and enum below must be in sync! */
FUNC *state[] = {entry_state, foo_state, bar_state, exit_state};

// return dst_state
static enum state_codes lookup_transitions(enum state_codes cur_state, enum ret_codes rc) 
{
    int i, len;
    printf("%s %d-%d\n", __func__, cur_state, rc);
    len = sizeof(state_transitions) / sizeof(state_transitions[0]);

    for (i = 0; i < len; i++) {
        if ((state_transitions[i].src_state == cur_state) 
            && (state_transitions[i].ret_code == rc)) {
            return state_transitions[i].dst_state;
        }
    }

    return fail_code;
}

int test_sm() {
    enum state_codes cur_state = ENTRY_STATE;
    enum ret_codes rc = CODE_OK;
    enum ret_codes (* state_fun)(void);

    for (;;) {
        printf("cur_state=%d rc=%d\n", cur_state, rc);
        state_fun = state[cur_state];
        rc = state_fun();
        if (EXIT_STATE == cur_state)
            break;
        cur_state = lookup_transitions(cur_state, rc);
		if (cur_state == fail_code) {
			printf("lookup fail!\n");
			break;
		}
    }

    return EXIT_SUCCESS;
}


// local test
//#define OFFLINE_TEST

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

#define ASRSTART        1
#define SYS_UNAUTH		0
#define SYS_AUTH		1

// robot current status
#define CURRENT_IDLE   0
#define CURRENT_PF   	1
#define CURRENT_VSLAM   2
#define CURRENT_FR   	3
#define CURRENT_MOVING 	4
#define CURRENT_OR 		5

using namespace std;
//static string result;
static bool sys_locked = false;
static bool speech_end = false;
static bool playing = false;
static int asr_flag = 0;
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;
static int current_sm = CURRENT_IDLE;

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

	ROS_INFO("+%s [%s]", __func__, session_begin_params);

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
#ifdef OFFLINE_TEST
{
	ROS_INFO("+%s fake g_result=%p\n", __func__, g_result);
	if (g_result) {
		free(g_result);
		g_result = NULL;
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	strcpy(g_result, "左");

	speech_end = false;
	speech_end = true;
	asr_flag = 1;
	sleep(5);
}
#else
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

	// file format: ֹͣ-2
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

	printf("-%s [%s] get code=%d\n", __func__, command, code);
	return code;
}

int main(int argc, char* argv[])
{
	geometry_msgs::Twist input_vel;

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

	// command for other modules
	ros::Publisher pub_cmd = n.advertise<std_msgs::Int32>("/voice/cmd_topic", 50);

	// control the robot
	ros::Publisher pub_robot = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000); 

	ros::Rate loop_rate(10);

	ROS_INFO("start listen ...");

	read_config();
	//std::cout << "start listen ..." << endl;
	while (ros::ok())
	{
		// listen .. 
		asrProcess();
		if (0)
		{	// rostopic pub -v -1 /mobile_base/commands/velocity geometry_msgs/Twist -- '[0.1,0,0]' '[0,0,0]'
			ROS_INFO("move...");
			input_vel.linear.x=0.0; //forward/back
			input_vel.linear.y=0.0;
			input_vel.linear.z=0.0;
			input_vel.angular.x=0.0;
			input_vel.angular.y=0.0;
			input_vel.angular.z=0.3; //left/right
			pub_robot.publish(input_vel);
			//continue;
		}
		// get voice result
		if (asr_flag)
		{
			std_msgs::String msg;
			std_msgs::Int32 cmd_msg;

			if ((g_result == NULL) || (strlen(g_result) < 2)) {
				ROS_INFO("no voice detected");
				continue;
			}

			// play back the received voice
			std_msgs::String msg_tts;
			// if system is locked, ignore voice input too
			if (sys_locked) 
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

			// code=0, stop all actions!

			if (code == 0) { // stop movement
				current_sm = CURRENT_IDLE;
				input_vel.linear.x=0.0; //forward/back
				input_vel.linear.y=0.0;
				input_vel.linear.z=0.0;
				input_vel.angular.x=0.0;
				input_vel.angular.y=0.0;
				input_vel.angular.z=0.0; //left/right
				pub_robot.publish(input_vel);
			}
			
			if (code >= 0) { // the voice is a special command
				cmd_msg.data = code;
				ROS_INFO("Publish command [%d]", code);

				memset(tts_content, 0, sizeof(tts_content));
				sprintf(tts_content, "执行命令 %s", g_result);
				msg_tts.data = tts_content;
				
				pub_tts.publish(msg_tts); // playback

				if (code <= 100) { // pf/vslam/or
					switch (code) {
						case 0:
							current_sm = CURRENT_IDLE;
							break;
						case 1:
							current_sm = CURRENT_PF;
							break;
						case 2:
							current_sm = CURRENT_IDLE;
							break;
						case 3:
							current_sm = CURRENT_VSLAM;
							break;
						case 4:
							current_sm = CURRENT_IDLE;
						case 5:
							current_sm = CURRENT_FR;
							break;
						default:
							break;
					}
					pub_cmd.publish(cmd_msg); // send to other modules
				} else { // control commands
					ROS_INFO("move %d...", code);
					input_vel.linear.x=0.0; //forward/back
					input_vel.linear.y=0.0;
					input_vel.linear.z=0.0;
					input_vel.angular.x=0.0;
					input_vel.angular.y=0.0;
					input_vel.angular.z=0.0; //left/right
					switch (code) {
						case 300:
							input_vel.angular.z=0.3;
							break;
						case 400:
							input_vel.angular.z=-0.3;
							break;
						case 500:
							input_vel.linear.x=0.1;
							break;
						case 600:
							input_vel.linear.x=-0.1;
							break;
						default:
							break;
					}
					pub_robot.publish(input_vel);
					current_sm = CURRENT_MOVING;
				}
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
		}else {
			ROS_INFO("asr_flag=false");
		}
		loop_rate.sleep();
        ros::spinOnce();
	 }

	return 0;
}
