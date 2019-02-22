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


float randomizePosition2()
{
  srand(5846 * time(NULL));
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
  Player(string player_name_in) // constructor
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
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("/bocas", 0);

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
    float sy = randomizePosition2();
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

  std::tuple<float, float> getDistanceAndAngleToPlayer(string player_to_get_distance)
  {
    tf::StampedTransform T0;

    try
    {
      listener.lookupTransform(player_name, player_to_get_distance, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();
      return {1000, 0};
    }

    float x = T0.getOrigin().x();
    float y = T0.getOrigin().y();
    float dist = sqrt(x * x + y * y);
    float ang = atan2(y, x);

    return {dist, ang};
  }

  std::tuple<float, float> getDistanceAndAngleToArena(string player_to_get_distance)
  {
    getDistanceAndAngleToPlayer("world");
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

    visualization_msgs::Marker marker;
    marker.header.frame_id = player_name;
    marker.header.stamp = ros::Time();
    marker.ns = player_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.y = 0.5;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // Step 2: Define local movement

    // for each prey, find the closest. Then, follow it
    vector<float> distance_to_preys;
    vector<float> angle_to_preys;

    for (size_t i = 0; i < msg->red_alive.size(); i++)
    {
      // ROS_WARN_STREAM("team_preys = " << msg->red_alive[i]);
      std::tuple<float, float> t = getDistanceAndAngleToPlayer(msg->red_alive[i]);
      distance_to_preys.push_back(std::get<0>(t));
      angle_to_preys.push_back(std::get<1>(t));
    }

    int idx_closest_prey = 0;
    float distance_closest_prey = 10000;
    for (size_t i = 0; i < team_preys->player_names.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_prey)
      {
        idx_closest_prey = i;
        distance_closest_prey = distance_to_preys[i];
      }
    }

    //hunters
    vector<float> distance_to_hunters;
    vector<float> angle_to_hunters;

    for (size_t i = 0; i < msg->green_alive.size(); i++)
    {
      std::tuple<float, float> t_hunt = getDistanceAndAngleToPlayer(msg->green_alive[i]);
      distance_to_hunters.push_back(std::get<0>(t_hunt));
      angle_to_hunters.push_back(std::get<1>(t_hunt));
    }

    int idx_closest_hunters = 0;
    float distance_closest_hunters = 10000;
    for (size_t i = 0; i < team_hunters->player_names.size(); i++)
    {
      if (distance_to_hunters[i] < distance_closest_hunters)
      {
        idx_closest_hunters = i;
        distance_closest_hunters = distance_to_hunters[i];
      }
    }

    float a;
    if (distance_closest_hunters > 1)
    {
      a = angle_to_hunters[idx_closest_hunters]+M_PI;
      marker.text = "RUNNING FROM " + team_hunters->player_names[idx_closest_hunters] + "!!!";
    }
    else
    {
      a = angle_to_preys[idx_closest_prey];
      marker.text = "RUN " + team_preys->player_names[idx_closest_prey] + "!!!";
    }

    std::tuple<float, float> t2 = getDistanceAndAngleToArena(player_name);
    if (std::get<0>(t2) > 5.5)
    {
      a = a + M_PI;
    }

    float dx = 10;

    // Step 2.5 : ckeck values
    float dx_max = msg->turtle;
    dx > dx_max ? dx = dx_max : dx = dx;

    double a_max = M_PI / 30;
    fabs(a) > fabs(a_max) ? a = a_max * a / fabs(a) : a = a;

    // change direction in case of leaving the arena

    // Step 3: Move
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, a);
    T1.setRotation(q);

    // Step 4: Define global movement
    tf::Transform Tg = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tg, ros::Time::now(), "world", player_name));

    vis_pub->publish(marker);
  }

private:
};

} // namespace drato_ns

main(int argc, char *argv[])
{
  ros::init(argc, argv, "drato");
  drato_ns::MyPlayer player("drato", "blue");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &drato_ns::MyPlayer::makeAPlayCallback, &player);

  player.printInfo();
  ros::Rate r(20);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
