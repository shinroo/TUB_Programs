#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/SensorPacket.h"
#include <create_fundamentals/StoreSongRequest.h>
#include <create_fundamentals/StoreSongResponse.h>
#include <ros/service_traits.h>



void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	ROS_INFO("number: %d, playing: %d", msg->songNumber, msg->songPlaying);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Song");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient PlaySong = n.serviceClient<create_fundamentals::PlaySong>("play_song");
  ros::ServiceClient StoreSong = n.serviceClient<create_fundamentals::StoreSong>("store_song");
  create_fundamentals::StoreSong srv;
  create_fundamentals::PlaySong srv1;

  srv.request.number = 1 ; 
  srv.request.song.push_back(64);
  srv.request.song.push_back(40*0.7);
  srv.request.song.push_back(30);
  srv.request.song.push_back(20*0.7);
  srv.request.song.push_back(55);
  srv.request.song.push_back(80*0.7);
  srv.request.song.push_back(30);
  srv.request.song.push_back(20*0.7);

  srv.request.song.push_back(60);
  srv.request.song.push_back(160*0.7); 
   srv.request.song.push_back(52);
  srv.request.song.push_back(20*0.7);
  srv.request.song.push_back(53);
  srv.request.song.push_back(160*0.7);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40*0.7);
  srv.request.song.push_back(67);
  srv.request.song.push_back(40*0.7);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40*0.7);
  StoreSong.call(srv);
  srv.request.number = 2 ;
  srv.request.song.push_back(69);
  srv.request.song.push_back(20*0.7);

//
 srv.request.song.push_back(71);
  srv.request.song.push_back(20*0.7); 
   srv.request.song.push_back(30);
  srv.request.song.push_back(20*0.7);
  srv.request.song.push_back(77);
  srv.request.song.push_back(160*0.7);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40);
  srv.request.song.push_back(69);
  srv.request.song.push_back(40);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40);

  srv.request.song.push_back(71);
  srv.request.song.push_back(20);

  srv.request.song.push_back(72);
  srv.request.song.push_back(80);

  srv.request.song.push_back(86);
  srv.request.song.push_back(80);

  srv.request.song.push_back(88);
  srv.request.song.push_back(80);
  srv.request.number=3 ; 
  StoreSong.call(srv);
 

 srv1.request.number = 1 ;
  
  PlaySong.call(srv1);	
  ros::Duration(2.0/64).sleep();
/*  srv.request.number = 2 ; 
  srv.request.song.push_back(76);
  srv.request.song.push_back(20);
  srv.request.song.push_back(77);
  srv.request.song.push_back(160);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40);
  srv.request.song.push_back(79);
  srv.request.song.push_back(40);
  srv.request.song.push_back(30);
  srv.request.song.push_back(40);

  srv.request.song.push_back(81);
  srv.request.song.push_back(20);
  StoreSong.call(srv);
*/
  srv1.request.number = 2 ;
   PlaySong.call(srv1);
 srv1.request.number = 3 ;
   PlaySong.call(srv1);
 ros::spin();
  return 0;
}
