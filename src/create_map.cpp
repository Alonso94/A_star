//
// Created by ali on 10/15/19.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace std;
using namespace cv;

pair<int,int> start_pose,goal_pose;
int cnt=0;

void callback(int event,int x,int y,int flags,void* userdata){
    Mat &image=*((Mat*)userdata);
    if(event== EVENT_LBUTTONDOWN && cnt==0){
        cout<<"start position "<<x<<" "<<y<<endl;
        start_pose=make_pair(x,y);
        circle(image,Point(x,y),10,CV_RGB(0,255,0),3);
        imshow("Resized map",image);
        cnt++;
    }
    else if(event==EVENT_LBUTTONDOWN && cnt==1){
        cout<<"goal position "<<x<<" "<<y<<endl;
        goal_pose=make_pair(x,y);
        circle(image,Point(x,y),10,CV_RGB(255,0,0),3);
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
    int w,h;
    cout<<"Enter the width and the height of the map (in meters):"<<endl;
//    cin>>w>>h;
    w=7;
    h=5;
    resize(image,image,Size(w*100,h*100));
    cout<<"define the start position and the goal on the map"<<endl;
    namedWindow("Resized map",WINDOW_AUTOSIZE);
    setMouseCallback("Resized map",callback,&image);
    imshow("Resized map",image);
    waitKey(0);
    return 0;
}
