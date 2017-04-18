#include "ras_target_recognition/object.hpp"

object::object(void){};
object::~object(void){};

object::object(std::string name)
{
    if(name == "red")
    {
      setHSVhigher(cv::Scalar(11,255,255));
      setHSVlower(cv::Scalar(0,200,156));
      setEsize(2);
      setDsize(2);
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
