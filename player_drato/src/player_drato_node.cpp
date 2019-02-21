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

#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace std;
using namespace boost;
using namespace ros;
// using namespace cv;

namespace drato_ns
{

class Team
{
  public:
    string team_name;
    vector<string>
        player_names;
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
    //Properties
    string player_name;

    //Methods
    Player(string player_name_in) //constructor
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

    string getTeamName() { return team_name; };

  private:
    string team_name = "";
    //     int a;
};

class MyPlayer : public Player
{
    boost::shared_ptr<Team> team_red;
    boost::shared_ptr<Team> team_green;
    boost::shared_ptr<Team> team_blue;
    boost::shared_ptr<Team> team_hunters;
    boost::shared_ptr<Team> team_mine;
    boost::shared_ptr<Team> team_preys;

  public:
    MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
    {
        team_red = (boost::shared_ptr<Team>)new Team("red");
        team_green = (boost::shared_ptr<Team>)new Team("green");
        team_blue = (boost::shared_ptr<Team>)new Team("blue");

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
        else {cout <<"Something wrong in team parametrization!!" << endl;}


        setTeamName(team_mine->team_name);
        printInfo();
    }

    void printInfo()
    {
        ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
        ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
    }

  private:
};

} // namespace drato_ns

main(int argc, char *argv[])
{

    ros::init(argc, argv, "drato");
    drato_ns::MyPlayer player("moliveira", "blue");
    ros::NodeHandle n;
    // string player_name = "drato";
    // player.setTeamName("blue");
    // player.setTeamName(2);

    // cout << "Hello world from " << player.player_name << " of team " << player.getTeamName() << endl;

    // drato_ns::Team team_blue("blue");
    // team_blue.player_names.push_back("drato");
    // team_blue.player_names.push_back("blourenco");

    while (ros::ok())
    {
        // cout << "Created an instance of class player with public name " << player_name << " of team " << player.getTeamName() << endl;
        // team_blue.printInfo();
        // cout << "drato belongs to team? " << team_blue.playerBelongsToTeam("drato") << endl;
        ros::Duration(1).sleep();
    }

    //cenas que nao interessam
    // ros::init(argc, argv, "player_drato_node");

    // ros::NodeHandle n;

    // for (int i = 0; i < 10; i++)
    // {
    //     cout << i << endl;
    // }

    return 0;
}
