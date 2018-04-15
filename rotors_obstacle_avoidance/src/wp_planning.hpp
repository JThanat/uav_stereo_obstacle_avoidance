#include <utility>
#include <vector>
#include <list>
#include <geometry_msgs/PoseStamped.h>

#include "umap_util.hpp"

using namespace std;

namespace wp
{
    typedef pair<double, double> iPair;
    double distance(double x1, double y1, double x2, double y2);
    bool has_collision(pair<double, double> p, vector<umap_utility::ellipse_desc> &obstacles, int obstacle_count);
    void waypoint_checking(vector<geometry_msgs::PoseStamped>& poses, vector<ellipse_desc> &obstacles, int obstacle_count, int num_wp,  int current_wp_idx);
}