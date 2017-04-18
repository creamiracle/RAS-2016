 #include "ras_object_detection/object.hpp"

object::object(void){};
object::~object(void){};

object::object(std::string name)
{
    if(name == "red")
    {
      setHSVhigher(cv::Scalar(3,255,255));
      setHSVlower(cv::Scalar(0,168,78));
      setEsize(1);
      setDsize(2);
      setcolor("red");
      setmsg_size1("I see a red ball");
      setmsg_size2("I see a red hollow cube");
      setmsg_size3("I see a red cube");
    }
    if(name == "green")
    {
      setHSVhigher(cv::Scalar(86,255,255));
      setHSVlower(cv::Scalar(40,100,40));
      setEsize(1);
      setDsize(0);
      setcolor("green");
      setmsg_size1("I see a green cylinder");
      setmsg_size2("I see a green cube");
    }
    if(name == "blue")
    {
      setHSVhigher(cv::Scalar(110,255,255));
      setHSVlower(cv::Scalar(91,100,20));
      setEsize(4);
      setDsize(0);
      setcolor("blue");
      setmsg_size1("I see a blue triangle");
      setmsg_size2("I see a blue cube");

    }
    if(name == "purple")
    {
      setHSVhigher(cv::Scalar(154,255,255));
      setHSVlower(cv::Scalar(125,50,50));
      setEsize(2);
      setDsize(1);
      setcolor("purple");
      setmsg_size1("I see a purple star");
      setmsg_size2("I see a purple cross");
    } 
    if(name == "yellow")
    {
      setHSVhigher(cv::Scalar(36,255,255));
      setHSVlower(cv::Scalar(15,200,130));
      setEsize(3);
      setDsize(3);
      setcolor("yellow");
      setmsg_size1("I see a yellow ball");
      setmsg_size2("I see a yellow cube");
    }    
    if(name == "orange")
    {
      setHSVhigher(cv::Scalar(15,255,255));
      setHSVlower(cv::Scalar(7,200,150));
      setEsize(2);
      setDsize(2);
      setcolor("orange");
      setmsg_size1("I see a orange star");
    }    
}
cv::Scalar object::getHSVhigher()
{
    return object::HSVhigher;
}

cv::Scalar object::getHSVlower()
{
    return object::HSVlower;
}

void object::setHSVhigher(cv::Scalar higher)
{
    object::HSVhigher = higher;
}

void object::setHSVlower(cv::Scalar lower)
{
    object::HSVlower = lower;
}
