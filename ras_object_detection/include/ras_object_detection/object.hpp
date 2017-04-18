#include <opencv/cv.h>
#include <string>
#include <vector>

class object
{
public:
    object(void);
    ~object(void);

    object(std::string name);

    cv::Scalar getHSVhigher();
    cv::Scalar getHSVlower();

    void setHSVhigher(cv::Scalar lower);
    void setHSVlower(cv::Scalar higher);

    int getEsize(){return Erosionsize ;}
    void setEsize(int Esize){ Erosionsize = Esize;}
    int getDsize(){return Dilationsize;}
    void setDsize(int Dsize){ Dilationsize = Dsize;}
    std::string getcolor(){return type;}
    void setcolor(std::string colorname)
    {
        type = colorname;
    }
    std::string getmsg_size1(){return message1;}
    void setmsg_size1(std::string msgs)
    {
        message1 = msgs;
    }
    std::string getmsg_size2(){return message2;}
    void setmsg_size2(std::string msgs)
    {
        message2 = msgs;
    }
    std::string getmsg_size3(){return message3;}
    void setmsg_size3(std::string msgs)
    {
        message3 = msgs;
    }
    void setXpos(double x) { xPos = x;}
    void setYpos(double y) { yPos = y;}
    void setZpos(double z) { zPos = z;}
    double getXpos(){return xPos;}
    double getYpos(){return yPos;}
    double getZpos(){return zPos;}

    std::vector<std::vector<double> > Objectpos;
    std::vector<cv::Mat> image;
    
        //std::string getType(){return type;}
        //std::string setType(std::string t){type = t;}
private:
    cv::Scalar HSVhigher;
    cv::Scalar HSVlower;
    std::string type,message1,message2,message3;
    int Erosionsize,Dilationsize;
    double xPos , yPos , zPos;
};
