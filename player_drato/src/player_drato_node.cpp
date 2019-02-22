/**
 * @file player_drato_node.cpp
 * @author Daniela Rato (danielarato@ua.pt)
 * @brief
 * @version 0.1
 * @date 2019-02-21
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace boost;
using namespace ros;
// using namespace cv;

float randomizePosition()
{
  srand(6832 * time(NULL));
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace drato_ns
{
class Team
{
public:
  string team_name;
  vector<string> player_names;
  ros::NodeHandle n;

  Team(string team_name_in)
  {
    team_name = team_name_in;
    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team " << team_name << " has players: " << endl;
    for (size_t i = 0; i < player_names.size(); cout << player_names[i++] << endl)
      ;
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_name == player_names[i])
      {
        // cout << "Belongs to team" << endl;
        return true;
      }
    }
    // cout << "Doesn't belong" << endl;
    return false;
  }

private:
};

class Player
{
public:
  // Properties
  string player_name;

  // Methods
  Player(string player_name_in)  // constructor
  {
    player_name = player_name_in;
  };

  void setTeamName(string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else
    {
      cout << "Cannot set team name" << team_name_in << endl;
      team_name = "";
    }
  }

  void setTeamName(int team_index)
  {
    if (team_index == 0)
      setTeamName("red");
    else if (team_index == 1)
      setTeamName("green");
    else if (team_index == 2)
      setTeamName("blue");
    else
      setTeamName("");
  }

  string getTeamName()
  {
    return team_name;
  };

private:
  string team_name = "";
};

class MyPlayer : public Player
{
  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vis_pub;

public:
  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");
    ros::NodeHandle n;
    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("player_names", 0);

    team_red->printInfo();
    team_green->printInfo();
    team_blue->printInfo();
    cout << player_name << endl;

    if (team_red->playerBelongsToTeam(player_name))
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }
    else if (team_green->playerBelongsToTeam(player_name))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }
    else if (team_blue->playerBelongsToTeam(player_name))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "Something wrong in team parametrization!!" << endl;
    }

    // define initial position
    float sx = randomizePosition();
    float sy = randomizePosition();
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(sx, sy, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    T1.setRotation(q);

    // Step 4: Define global movement
    tf::Transform Tg = T1;
    br.sendTransform(tf::StampedTransform(Tg, ros::Time::now(), "world", player_name));

    ros::Duration(0.1).sleep();

    setTeamName(team_mine->team_name);
    printInfo();
  }

  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
    ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new msg");

    // Step 1: find out where I am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("/world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    // Step 2: Define local movement
    float dx = 0.5;
    float a = -M_PI;

    // Step 2.5 : ckeck values
    float dx_max = msg->turtle;
    dx > dx_max ? dx = dx_max : dx = dx;

    double a_max = M_PI / 30;
    fabs(a) > fabs(a_max) ? a = a_max * a / fabs(a) : a = a;

    // Step 3: Move
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, a);
    T1.setRotation(q);

    // Step 4: Define global movement
    tf::Transform Tg = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tg, ros::Time::now(), "world", player_name));

    visualization_msgs::Marker marker;
    marker.header.frame_id = player_name;
    marker.header.stamp = ros::Time();
    marker.ns = player_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 0.6;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.text = player_name;
    vis_pub->publish(marker);
  }

private:
};

}  // namespace drato_ns

main(int argc, char *argv[])
{
  ros::init(argc, argv, "drato");
  drato_ns::MyPlayer player("drato", "blue");
  ros::NodeHandle n;

  // string player_name = "drato";
  // player.setTeamName("blue");
  // player.setTeamName(2);
  // cout << "Hello world from " << player.player_name << " of team " << player.getTeamName() << endl;
  // drato_ns::Team team_blue("blue");
  // team_blue.player_names.push_back("drato");

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &drato_ns::MyPlayer::makeAPlayCallback, &player);

  player.printInfo();
  ros::Rate r(20);

  while (ros::ok())
  {
    // cout << "Created an instance of class player with public name " << player_name << " of team " <<
    // player.getTeamName() << endl; team_blue.printInfo(); cout << "drato belongs to team? " <<
    // team_blue.playerBelongsToTeam("drato") << endl;
    // ros::Duration(0.1).sleep();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


//por jogador a preseguir alguem e a fugir de alguem
//ver presas e calcular distancias
//escolher presa mais proxima e preseguir 