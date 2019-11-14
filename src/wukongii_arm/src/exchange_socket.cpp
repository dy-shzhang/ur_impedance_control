#include <ros/ros.h>
#include <stdio.h>   
#include <string.h>   
#include <errno.h>   
#include <stdlib.h>   
#include <unistd.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>   
   
  
#define DEST_PORT 2001   
#define DSET_IP_ADDRESS  "192.168.1.3"   

struct cmd{
    double position;
    double velocity;
    bool have_velocity;
};   
  
int main(int argc,char** argv)  
{  
    ros::init(argc, argv,"socket_client_to_kc",ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();

  /* socket文件描述符 */  
  int sock_fd;  
  
  /* 建立udp socket */  
  sock_fd = socket(AF_INET, SOCK_DGRAM, 0);  //SOCK_DGRAM UPD 不可靠,无连接的,定长
  if(sock_fd < 0)  
  {  
    ROS_INFO("socket, unable to connect to server"); 
    exit(1);  
  }  
    
  /* 设置address */  
  struct sockaddr_in addr_serv;  
  int len;  
  memset(&addr_serv, 0, sizeof(addr_serv));  //将某一内存块儿设定为统一值

  addr_serv.sin_family = AF_INET;  
  addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);  
  addr_serv.sin_port = htons(DEST_PORT);  
  len = sizeof(addr_serv);  


  int recv_len;
  int send_num;  
  int recv_num;  
  char send_buf[20] = "hey, who are you?\n";  
  char recv_buf[20];  
  ros::Rate loop_rate(500);
  while(ros::ok()){
      
        printf("client send: %s\n", send_buf);  
    
        send_num = sendto(sock_fd, send_buf, strlen(send_buf), 0, (struct sockaddr *)&addr_serv, len);  
        ROS_INFO("%d", send_num);
        if(send_num < 0){  
            perror("sendto error:");  
            exit(1);
        }  
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_serv, (socklen_t *)&len);  
        //recv_num = recv(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL, (struct sockaddr *)&addr_serv, (socklen_t *)&recv_len); 
        ROS_INFO("%d", recv_num);
        if(recv_num < 0) {  
            perror("recvfrom error:");  
            exit(1);  
        }     
        recv_buf[recv_num] = '\0';  
        printf("client receive %d bytes: %s\n", recv_num, recv_buf);  
        ros::spinOnce();
        loop_rate.sleep();
    }  
    close(sock_fd);  
    ros::waitForShutdown(); 
    return 0;  
}