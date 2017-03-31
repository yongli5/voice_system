#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>
#include <exception>

using namespace std;
string result;
int flag = 0;

int writer(char *data, size_t size, size_t nmemb, string *writerdata)
{
    if (writerdata == NULL)
    {
       return -1;
    }
    int len = size*nmemb;
    writerdata->append(data, len);

    return len;
}

int parseJsonResponse(string input)
{
    Json::Value root;
    Json::Reader reader;
    bool parsedone = reader.parse(input, root);

    if(!parsedone) {
       cout << "Failed to parse the response data !" << endl;
       return -1;
    }
    const Json::Value code = root["code"];
    const Json::Value text = root["text"];

    result = text.asString();
    flag = 1;
    cout << "response code: " << code << endl;
    cout << "response text: " << code << endl;

    return 0;
}

int HttpPostRequest(string input)
{
     string buffer;

     std::string strJson = "{";
     strJson += "\"key\" : \"7086890091eb41bf9f39242a78f0eed3\",";
     strJson += "\"info\" : ";
     strJson += "\"";
     strJson += input;
     strJson += "\"";
     strJson += "}";

     cout<< "post json string: " << strJson <<endl;
     try {
        CURL *pcurl = NULL;
	CURLcode res;
	curl_global_init(CURL_GLOBAL_ALL);

	pcurl = curl_easy_init();
        if (NULL != pcurl) {
	   //set url timeout
	   curl_easy_setopt(pcurl, CURLOPT_TIMEOUT, 8);
          // set url that is about to receive the POST request 
	   curl_easy_setopt(pcurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");
	   // set curl http header
	   curl_slist *plist = curl_slist_append(NULL, "Content-Type:application/json; charset=utf-8");
	   curl_easy_setopt(pcurl, CURLOPT_HTTPHEADER, plist);
	
	   curl_easy_setopt(pcurl, CURLOPT_POSTFIELDS, strJson.c_str());

	   curl_easy_setopt(pcurl, CURLOPT_WRITEFUNCTION, writer);  
           curl_easy_setopt(pcurl, CURLOPT_WRITEDATA, &buffer);

	   res = curl_easy_perform(pcurl);

	   if (res != CURLE_OK) {
	      printf("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));  
	   
	   }
	   curl_easy_cleanup(pcurl);
	}
        
	curl_global_cleanup();
     }
     catch (std::exception &ex) {
        printf("curl exception !!", ex.what());     
     }

     if (buffer.empty()) {
        printf("ERROR! The tuling server response NULL\n");
     
     } else {
       parseJsonResponse(buffer); 
     }

     return 0;
}

void nlpCallback(const std_msgs::String::ConstPtr& msg)
{
     std::cout<<"Your question is: "<< msg->data << std::endl; 

     HttpPostRequest(msg->data);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tuling_nlu_node");

	ros::NodeHandle n;

	// published from ASR
	ros::Subscriber sub = n.subscribe("/voice/tuling_nlu_topic", 5, nlpCallback);

    // publish to tts and play it
	ros::Publisher pub = n.advertise<std_msgs::String>("/voice/xf_tts_topic", 10);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		if (flag)
		{
			std_msgs::String msg;	
			msg.data = result;
			pub.publish(msg);
			flag =0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	 }

	return 0;
}

