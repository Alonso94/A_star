//
// Created by ali on 10/15/19.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stack>

using namespace std;
using namespace cv;

#define x first
#define y second

typedef pair<int,int> pose;
//<f,<x,y>>
typedef pair<double,pose> f_pose;
pose start_pose,goal_pose;
int cnt=0;


class A_star{
private:
    pose start_pose,goal_pose;
    Mat map,array;

    struct Node{
        pose parent;
        double f,g,h;
    };

    Node **node_array;
public:
    A_star(Mat* colored_map,Mat* bi_map,pose start_position,pose goal_position){
        map=*colored_map;
        array=*bi_map;
        start_pose=start_position;
        goal_pose=goal_position;
        node_array=new Node*[array.rows];
        for(int i=0;i<array.rows;i++)
            node_array[i]=new Node[array.cols];
        namedWindow("Planning",WINDOW_NORMAL);
        resizeWindow("Planning",700,500);
        imshow("Planning",map);
        waitKey(4);

    }
    bool check_boundaries(pose coord){
        return (coord.x>=0) && (coord.x<array.cols) && (coord.y>=0) && (coord.y<array.rows);
    }
    bool check_available(pose coord){
        // not obstacle
        return (int)array.at<uchar>(coord.y,coord.x)==0;
    }
    bool check_goal(pose coord){
        return (coord.x==goal_pose.x) && (coord.y==goal_pose.y);
    }
    double calc_h(pose coord){
        return (double) sqrt(pow(coord.x-goal_pose.x,2)+pow(coord.y-goal_pose.y,2));
    }
    void draw(pose coord){
        map.at<Vec3b>(coord.y,coord.x)=Vec3b(255,255,0);
        imshow("Planning",map);
        waitKey(1);
    }
    void draw_path(stack<pose> path){
        while(!path.empty()){
            pose p=path.top();
            path.pop();
            map.at<Vec3b>(p.y,p.x)=Vec3b(0,255,0);
            imshow("Planning",map);
            waitKey(1);
        }
        waitKey(0);
    }
    stack<pose> build_path(){
        int i,j,new_i,new_j;
        i=goal_pose.x;
        j=goal_pose.y;
        stack<pose> path;
        while(true){
            path.push(make_pair(i,j));
            new_i=node_array[i][j].parent.x;
            new_j=node_array[i][j].parent.y;
            if (new_i==i && new_j==j) break;
            i=new_i;
            j=new_j;
        }
        draw_path(path);
        return path;
    }
    void planning(){
        bool closed_list[array.rows][array.rows];
        memset(closed_list,false,sizeof(closed_list));
        for(int i=0;i<array.rows;i++)
            for(int j=0;j<array.cols;j++){
                node_array[i][j].f=FLT_MAX;
                node_array[i][j].g=FLT_MAX;
                node_array[i][j].h=FLT_MAX;
                node_array[i][j].parent=make_pair(-1,-1);
            }
        int x_,y_;
        x_=start_pose.x;
        y_=start_pose.y;
        node_array[x_][y_].f=0.0;
        node_array[x_][y_].g=0.0;
        node_array[x_][y_].h=0.0;
        node_array[x_][y_].parent=make_pair(x_,y_);

        set<f_pose> open_list;
        // <f,<x,y>>
        open_list.insert(make_pair(0.0,make_pair(x_,y_)));
        int dx[]={-1,+1,0,0,-1,-1,+1,+1};
        int dy[]={0,0,+1,-1,+1,-1,+1,-1};
        double new_f,new_g,new_h;

        bool goal_reached=false;

        while(!open_list.empty()){
            f_pose p=*open_list.begin();
            open_list.erase(open_list.begin());
            x_=p.y.x;
            y_=p.y.y;
            //cout<<x_<<" "<<y_<<endl;
            for(int i=0;i<8;++i){
                //cout<<i<<endl;
                pose test_p=make_pair(x_+dx[i],y_+dy[i]);
                if(check_boundaries(test_p)){
                    if(check_goal(test_p)){
                        node_array[test_p.x][test_p.y].parent=make_pair(x_,y_);
                        build_path();
                        goal_reached=true;
                        return;
                    }
                    else if(check_available(test_p) && !closed_list[test_p.x][test_p.y]){
                        new_g=node_array[x_][y_].g+ 1.0;
                        new_h=calc_h(test_p);
                        new_f=new_g+new_h;
                        if(node_array[test_p.x][test_p.y].f==FLT_MAX || node_array[test_p.x][test_p.y].f>new_f){
                            open_list.insert(make_pair(new_f,test_p));
                            draw(test_p);
                            node_array[test_p.x][test_p.y].f=new_f;
                            node_array[test_p.x][test_p.y].g=new_g;
                            node_array[test_p.x][test_p.y].h=new_h;
                            node_array[test_p.x][test_p.y].parent=make_pair(x_,y_);
                        }
                    }
                }
            }
        }
        if(!goal_reached)
            cout<<"goal not reached";
   }
};

void callback(int event,int x,int y,int flags,void* userdata){
    Mat &image=*((Mat*)userdata);
    if(event== EVENT_LBUTTONDOWN && cnt==0){
        cout<<"start position "<<x<<" "<<y<<endl;
        start_pose=make_pair(x,y);
        image.at<Vec3b>(y,x)=Vec3b(0,255,0);
        imshow("Resized map",image);
        cnt++;
    }
    else if(event==EVENT_LBUTTONDOWN && cnt==1){
        cout<<"goal position "<<x<<" "<<y<<endl;
        goal_pose=make_pair(x,y);
        image.at<Vec3b>(y,x)=Vec3b(0,0,255);
        imshow("Resized map",image);
        cnt++;
    }
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"create_map");
    ros::NodeHandle n;
    string name;
    cout<<"enter the name of the image:"<<endl;
    //cin>>name;
    name="map.png";
    string path=ros::package::getPath("A_star")+"/src/"+name;
    Mat image=imread(path,CV_LOAD_IMAGE_COLOR);
//    namedWindow("Map",WINDOW_AUTOSIZE);
//    imshow("Map",image);
//    waitKey(0);
//    destroyWindow("Map");
    int w,h,s;
    cout<<"Enter the width and the height of the map (in meters):"<<endl;
//    cin>>w>>h;
    cout<<"Enter discretization step (in cm):"<<endl;
//    cin>>s;
    s=10;
    w=7;
    h=5;
    resize(image,image,Size(w*100/s,h*100/s));
    cout<<"define the start position and the goal on the map"<<endl;
    namedWindow("Resized map",WINDOW_NORMAL);
    resizeWindow("Resized map",700,500);
    setMouseCallback("Resized map",callback,&image);
    imshow("Resized map",image);
    waitKey(0);
    destroyWindow("Resized map");
    Mat gray,binary;
    cvtColor(image,gray,CV_RGB2GRAY);
    threshold(gray,binary,240,255,THRESH_BINARY_INV);
//    (int)binary.at<uchar>(0,0)
    A_star a(&image,&binary,start_pose,goal_pose);
    a.planning();
    return 0;
}
