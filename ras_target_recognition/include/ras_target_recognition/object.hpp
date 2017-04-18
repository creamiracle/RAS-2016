#include <cv.h>
#include <string>

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

        //std::string getType(){return type;}
        //std::string setType(std::string t){type = t;}
private:
    cv::Scalar HSVhigher;
    cv::Scalar HSVlower;
    std::string type;
    int Erosionsize,Dilationsize;
};
