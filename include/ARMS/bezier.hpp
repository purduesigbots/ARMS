#include "point.h"
#include <vector>
using namespace arms;
struct Pose{
    Point pos; double heading;
    std::vector<double> returnPose(){
        return {pos.x,pos.y,heading};
    }
};

class Bezier {
    private:
        double length = 0;
        std::vector<Pose> pose_list; 
        //Point getPoint(double t);
    public:
        Point points[4];
        
        Bezier(Point p0, Point p1, Point p2, Point p3) {
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;
            points[3] = p3;
        }
        

Point getPoint(double t);

Point getVelocity(double t);



double getHeading(double t);

double getLength(double t);
void getPoses();
Pose getPose(double t);
Point getPointAtLength(double length);

double getClosestPoint(Point p);
int get_point_num();
std::vector<Pose> returnPoseList();



};