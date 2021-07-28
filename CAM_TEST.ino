
#include "Seeed_Arduino_TinyCV.h"
#include "Arduino.h"
#include <iostream>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "dma.h"
#include "dcmi.h"
#include "OV2640.h"
#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

using namespace std;
using namespace cv;

EXTMEM uint8_t cam_buf[320*240*2];

static Frame_TypeDef frame = {
    .buffer = cam_buf,
    .length = 320*240*2,
    .width  = 320,
    .height = 240
};
static OV2640_TypeDef OV2640 = {
    .dcmi = &hdcmi,
    .dma = &hdma_dcmi_pssi,
    .cam_i2c = &Wire,
    .frame = &frame,
    .fps = 0,
    .show = 0
};

class ShapeDetector
{
public:
    ShapeDetector();
    ~ShapeDetector();
    void detect(const Mat &curve);
    string get_shape_type();

private:
    string m_shape;
};

ShapeDetector::ShapeDetector()
{
}

ShapeDetector::~ShapeDetector()
{
}

void ShapeDetector::detect(const Mat &curve)
{
    string shape = "unidentified";
    double peri = arcLength(curve, true);
    Mat approx;
    approxPolyDP(curve, approx, 0.04 * peri, true); // 0.01~0.05
    const int num_of_vertices = approx.rows;

    // if the shape is a triangle, it will have 3 vertices
    if (num_of_vertices == 3)
    {
        shape = "triangle";
    }
    else if (num_of_vertices == 4)
    { // if the shape has 4 vertices, it is either a square or a rectangle
        // Compute the bounding box of the contour and
        // use the bounding box to compute the aspect ratio
        Rect rec = boundingRect(approx);
        double ar = 1.0 * rec.width / rec.height;

        // A square will have an aspect ratio that is approximately
        // equal to one, otherwise, the shape is a rectangle
        if (ar >= 0.95 && ar <= 1.05)
        {
            shape = "square";
        }
        else
        {
            shape = "rectangle";
        }
    }
    else if (num_of_vertices == 5)
    { // if the shape is a pentagon, it will have 5 vertices
        shape = "pentagon";
    }
    else
    { // otherwise, we assume the shape is a circle
        shape = "circle";
    }
    m_shape = shape;
}

string ShapeDetector::get_shape_type()
{
    return m_shape;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_RED, OUTPUT);
  Wire.begin();
  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  Serial.begin(115200);
  Serial.printf("CAM TEST BEGIN\n\r");
  OV2640_Init(&OV2640);
  OV2640_ReadID(&(OV2640.ID));
  Serial.printf("ID: %02X %02X \r\n", OV2640.ID.PIDH, OV2640.ID.PIDL);
  OV2640_UXGAConfig();
  OV2640_Start();

}

void loop() 
{
  if(OV2640.show)
  {
    
     OV2640.show = 0;
     uint32_t runTime = millis();
     Mat image(240, 320, CV_8UC2, cam_buf); //creat a mat 
     Mat gray;
     cvtColor(image, gray, COLOR_BGR5652GRAY);
     Mat blurred, thresh;
     GaussianBlur(gray, blurred, Size(5, 5), 0.0);
     threshold(blurred, thresh, 60, 255, THRESH_BINARY);
     vector<vector<Point>> contours;
     vector<Vec4i> hierarchy;
     findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
     ShapeDetector sd;
     vector<Point> c;
     for (size_t i = 0; i < contours.size(); i++)
     {
        c = contours[i];
        Rect crect = boundingRect(c);
        // compute the center of the contour, then detect the name of the
        // shape using only the contour
        Moments M = moments(c);
        int cX = static_cast<int>(M.m10 / M.m00);
        int cY = static_cast<int>(M.m01 / M.m00);
        sd.detect(Mat(c));
        string shape = sd.get_shape_type();
        drawContours(image, contours, i, Scalar(0, 255, 0), 2);
        Point pt(cX, cY);
        putText(image, shape, pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
     }
     Serial.printf("runTime is %ld\n\r", millis() - runTime);
     imshow("asd", image);
     OV2640_Start();
  }
}
