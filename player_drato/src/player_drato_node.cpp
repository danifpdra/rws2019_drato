#include <iostream>

#include <ros/ros.h>

#include <vector>

using namespace std;
// using namespace cv;

namespace drato_ns
{

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
  public:
    MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
    {
        setTeamName(team_name_in);
    }

  private:
};

class Team
{
  public:
    string team_name;
    vector<string>
        player_names;

    Team(string team_name_in)
    {
        team_name = team_name_in;
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

} // namespace drato_ns

main(int argc, char *argv[])
{
    drato_ns::MyPlayer player("drato", "blue");
    // Player player("drato");
    // player.setTeamName("blue");
    // player.setTeamName(2);

    cout << "Hello world from " << player.player_name << " of team " << player.getTeamName() << endl;

    drato_ns::Team team_blue("blue");
    team_blue.player_names.push_back("drato");
    team_blue.player_names.push_back("blourenco");

    team_blue.printInfo();
    cout << "drato belongs to team? " << team_blue.playerBelongsToTeam("drato") << endl;

        //cenas que nao interessam
        // ros::init(argc, argv, "player_drato_node");

        // ros::NodeHandle n;

        // for (int i = 0; i < 10; i++)
        // {
        //     cout << i << endl;
        // }

        return 0;
}
