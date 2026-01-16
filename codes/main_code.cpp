#include<wiringPi.h>
#include<stdio.h>
#include<sys/time.h>
#include<lcd.h>
#include<wiringSerial.h>
#include<stdlib.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<time.h>
#include<fstream>
#include<stdlib.h>
#include<softPwm.h>

#define USS_TRIG 4
#define USS_ECHO 5
#define LED_RED 27
#define LED_BLUE 26
#define LCD_RS 29
#define LCD_E 28
#define LCD_D4 25
#define LCD_D5 24
#define LCD_D6 23
#define LCD_D7 22
#define SERVO_MOTOR 1
#define BASIC_SPEED 20

int MOTOR_SPEED=BASIC_SPEED;

using namespace std;
using namespace cv;

void doTask();
void calPid();
void gogogo();
void stop();
void playMusic();
void imgRecognition();
void ledBlinking();
void pinSetup();
void cameraUp(double up);
void lineFollowing(double mainROIPercent,int display,double resizeScale);
float getDistance();
void stopAtDistance(float distance);
void shortCut(double mainROIPercent,int display,double resizeScale,int colour); // 1.red(slow) 2.yellow(normal) 3.green(fast) 4.blue(split)
void loadImg(string loadPath);
int matchTemp();
void findApprox(vector<Point>& corners);
int processImage(Mat& img);
int wait4Traffic();
void kickFootball();
vector<Mat> importedImages;
unsigned char triangleChar[8]={
	0b00000,
	0b00100,
	0b00100,
	0b01010,
	0b01010,
	0b10001,
	0b11111,
	0b00000
};
unsigned char squareChar[8]={
	0b00000,
	0b00000,
	0b11111,
	0b10001,
	0b10001,
	0b10001,
	0b11111,
	0b00000
};
unsigned char circleChar[8]={
	0b00000,
	0b00000,
	0b01110,
	0b10001,
	0b10001,
	0b10001,
	0b01110,
	0b00000
};
float Kp=3,Ki=0,Kd=0;
float P=0,I=0,D=0,pidValue=0;
float pidError=0,lastError=0,lastI=0;
char instruction[23];
int leftSpeed,rightSpeed;
int robot;
int lcd;
int afterYellow=0;
int afterFootball=0;

int main(){
    pinSetup();
	robot=serialOpen("/dev/ttyAMA0",57600); // returns int, -1 for error
	if(robot==-1){
		puts("error in openning robot");
		return -1;
	}
    puts("successfully opened");
    serialPuts(robot,"#ha");
    lcd=lcdInit(2,16,4,LCD_RS,LCD_E,LCD_D4,LCD_D5,LCD_D6,LCD_D7,0,0,0,0);
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    loadImg("KIRIN_PIC/");
    lcdPrintf(lcd,"Ready!");

    int cp=0;
    cin>>cp;
    for(int i=0;i<10;i++){
        lineFollowing(0.7,1,0.5);
        stop();
    }
    doTask();
    for(int i=0;i<10;i++){
        lineFollowing(0.7,1,0.5);
        stop();
    }
    doTask();
    lineFollowing(0.7,1,0.5);


    return 0;
}

void doTask(){

    cout<<"do task"<<endl;
	cameraUp(1);
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"Recognising");
    lcdCharDef(lcd,10,triangleChar);
    lcdCharDef(lcd,11,squareChar);
    lcdCharDef(lcd,12,circleChar);
    int taskNum=matchTemp();
    switch(taskNum){
        case 1:
            cout<<"play music"<<endl;
            lcdClear(lcd);
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"Play Music!");
            playMusic();
            break;
        case 2:
            cout<<"led blinking"<<endl;
            lcdClear(lcd);
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"LED Blinking!");
            ledBlinking();
            break;
        case 3:
            cout<<"wait for traffic"<<endl;
            lcdClear(lcd);
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"Wait Traffic!");
            wait4Traffic();
            break;
        case 4:
            cout<<"shortcut yellow"<<endl;
            shortCut(0.7,1,0.5,2);
            break;
        case 5:
            cout<<"shortcut red"<<endl;
            shortCut(0.7,1,0.3,1);
            break;
        case 6:
            cout<<"shortcut green"<<endl;
            shortCut(0.3,1,0.5,3);
            break;
        case 7:
            cout<<"shortcut blue"<<endl;
            shortCut(0.7,1,0.5,4);
            break;
        case 8:
            cout<<"stop at 5cm"<<endl;
            lcdClear(lcd);
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"Stop at 5cm!");
            stopAtDistance(7.3);
			delay(1000);
			serialPuts(robot,"#Barrrr 020 020 020 020");
			delay(2000);
			stop();
            break;
        case 9:
            cout<<"kick football"<<endl;
            lcdClear(lcd);
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"Kick football!");
            kickFootball();
            break;
        case 10:
            cout<<"t:3 s:2 c:3"<<endl;
			lcdClear(lcd);
			lcdPosition(lcd,0,0);
			lcdPuts(lcd,"Count Shapes!");
			lcdPosition(lcd,0,1);
			lcdPutchar(lcd,10);
			lcdPrintf(lcd,"3 ");
			lcdPutchar(lcd,11);
			lcdPrintf(lcd,"2 ");
			lcdPutchar(lcd,12);
			lcdPrintf(lcd,"3 ");
			delay(1500);
            break;
        case 11:
            cout<<"t:2 s:1 c:2"<<endl;
			lcdClear(lcd);
			lcdPosition(lcd,0,0);
			lcdPuts(lcd,"Count Shapes!");
			lcdPosition(lcd,0,1);
			lcdPutchar(lcd,10);
			lcdPrintf(lcd,"2 ");
			lcdPutchar(lcd,11);
			lcdPrintf(lcd,"1 ");
			lcdPutchar(lcd,12);
			lcdPrintf(lcd,"2 ");
			delay(1500);
            break;
        case 12:
            cout<<"t:2 s:2 c:2"<<endl;
			lcdClear(lcd);
			lcdPosition(lcd,0,0);
			lcdPuts(lcd,"Count Shapes!");
			lcdPosition(lcd,0,1);
			lcdPutchar(lcd,10);
			lcdPrintf(lcd,"2 ");
			lcdPutchar(lcd,11);
			lcdPrintf(lcd,"2 ");
			lcdPutchar(lcd,12);
			lcdPrintf(lcd,"2 ");
			delay(1500);
            break;
        default:
            //pass
            cout<<"I Don't Know!"<<endl;
			lcdClear(lcd);
			lcdPosition(lcd,0,0);
			lcdPuts(lcd,"I Don't Know!");
            break;
    }
	cameraUp(0);
}

void pinSetup(){
    wiringPiSetup();
    pinMode(SERVO_MOTOR,OUTPUT);
	pinMode(LED_RED,OUTPUT);
	pinMode(LED_BLUE,OUTPUT);
    pinMode(USS_TRIG,OUTPUT);
    pinMode(USS_ECHO,INPUT);
    //pinMode(SERVO_MOTOR,PWM_OUTPUT);
    softPwmCreate(SERVO_MOTOR,0,100);
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_BLUE,LOW);
    //pwmSetMode(PWM_MODE_MS);
    //pwmSetClock(192);
    //pwmSetRange(2000);
}


void cameraUp(double up){
    if(up==1){
        //pwmWrite(SERVO_MOTOR,163); // MAX100,MIN235,MID150
        softPwmWrite(SERVO_MOTOR,16);
    }
    else if(up==0){
        //pwmWrite(SERVO_MOTOR,222); // MAX100,MIN235,MID150
        softPwmWrite(SERVO_MOTOR,22.8);
    }
    else if(up<1&&up>0){
        //pwmWrite(SERVO_MOTOR,up); // MAX100,MIN235,MID150
        softPwmWrite(SERVO_MOTOR,22.8-up*6.8);
    }
    else{
        cout<<"not a proper value for servo pwm"<<endl;
    }
    delay(800);
    softPwmWrite(SERVO_MOTOR,0);
    //pwmWrite(SERVO_MOTOR,0);
}

void lineFollowing(double mainROIPercent,int display,double resizeScale){
	cameraUp(0);
    int distance;
	VideoCapture cap(0);
    if(!cap.isOpened()){
        cout<<"Error opening camera"<<endl;
        return;
    }
	//cap.set(CAP_PROP_FRAME_WIDTH,1280*resizeScale);
	//cap.set(CAP_PROP_FRAME_HEIGHT,720*resizeScale);
	//cap.set(CAP_PROP_FPS,30);
    int capWidth=(int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int capHeight=(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	Mat frame;
	Mat orgFrame;
	int stopAtMagenta=0;
    timespec lineFollowingBegin,lineFollowingCheckPoint;
    clock_gettime(CLOCK_MONOTONIC,&lineFollowingBegin);
    Scalar lowerBlack=Scalar(0,0,0);
    Scalar upperBlack=Scalar(180,255,50);      // MAX (180,255,255)
    Scalar lowerMagenta=Scalar(130,50,50);
    Scalar upperMagenta=Scalar(170,255,255);
    int waitMagentaTime=1200000;
	while(1){
        if(!stopAtMagenta){
            clock_gettime(CLOCK_MONOTONIC,&lineFollowingCheckPoint);
            double timeInCheckPoint=(lineFollowingCheckPoint.tv_sec-lineFollowingBegin.tv_sec)*1000000;
            if(afterFootball==1){
                waitMagentaTime=3000000;
                afterFootball=0;
            }
            else{
                waitMagentaTime=1200000;
            }
            if(afterYellow==1){
                stopAtMagenta=1;
                afterYellow=0;
            }
            else if(timeInCheckPoint>waitMagentaTime&&afterYellow==0){
                stopAtMagenta=1;
            }
        }
        else{
            lcdPosition(lcd,10,1);
            lcdPrintf(lcd,"flag");
        }
		cap>>orgFrame;
		resize(orgFrame,frame,Size(capWidth*resizeScale,capHeight*resizeScale));
		Mat hsv;
		cvtColor(frame,hsv,COLOR_BGR2HSV);
		Point frameCentre(frame.cols/2,frame.rows/2);        // centre to be reference
        Rect mainROI(0,(frame.rows-frame.rows*mainROIPercent)/2,frame.cols,frame.rows*mainROIPercent);   // MAINrecognition range
        Mat recognition=hsv(mainROI);
        Mat magentaMask;
        inRange(recognition,lowerMagenta,upperMagenta,magentaMask);
        vector<vector<Point>> magentaContours;
        vector<Vec4i> magentaHierarchy;
        findContours(magentaMask,magentaContours,magentaHierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        double magentaContoursMaxArea=0;
        int magentaContoursMaxAreaI=-1;
        for(int i=0;i<magentaContours.size();i++){
            double area=contourArea(magentaContours[i]);
            if(area>magentaContoursMaxArea){
                magentaContoursMaxArea=area;
                magentaContoursMaxAreaI=i;
            }
        }
        if(magentaContoursMaxAreaI!=-1){
            vector<vector<Point>> magentaApproxList;
            vector<Point> magentaApprox;
            approxPolyDP(magentaContours[magentaContoursMaxAreaI],magentaApprox,4,true);
            magentaApproxList.push_back(magentaApprox);
            Rect magentaRect=boundingRect(magentaContours[magentaContoursMaxAreaI]);
            if(magentaRect.area()>=magentaContoursMaxArea*0.63
                &&magentaRect.area()<=magentaContoursMaxArea*1.38
                &&frameCentre.y-magentaRect.y<=frame.rows/3
                &&stopAtMagenta
                ){
                    serialPuts(robot,"#ha");
                    lcdClear(lcd);
                    lcdPosition(lcd,0,0);
                    lcdPuts(lcd,"stop!");
                    cameraUp(1);
                    break;
            }
//            else if(magentaRect.area()>=magentaContoursMaxArea*1.5
//                &&frameCentre.y-magentaRect.y<=frame.rows/3
//                ){
//                    pidError=114.5;
//                    calPid();
//                    gogogo();
//                    continue;
//            }
            if(display){
                drawContours(frame,magentaApproxList,0,Scalar(0,255,0),2,LINE_8,magentaHierarchy,0,Point(mainROI.x,mainROI.y));
                rectangle(frame,Point(magentaRect.x+mainROI.x,magentaRect.y+mainROI.y),Point(magentaRect.x+mainROI.x+magentaRect.width,magentaRect.y+mainROI.y+magentaRect.height),Scalar(0,0,255),2,LINE_8);
            }
        }
        Mat mask;
        inRange(recognition,lowerBlack,upperBlack,mask);
		vector<vector<Point>> blackContours;
        vector<Vec4i> hierarchy;
        findContours(mask,blackContours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        double blackContoursMaxArea=0;
        int blackContoursMaxAreaI=-1;
        for(int i=0;i<blackContours.size();i++){
            double area=contourArea(blackContours[i]);
            if(area>blackContoursMaxArea){
                blackContoursMaxArea=area;
                blackContoursMaxAreaI=i;
            }
        }
		if(blackContoursMaxAreaI!=-1){
            Moments mu=moments(blackContours[blackContoursMaxAreaI]);
            Point centre(mu.m10/mu.m00,mu.m01/mu.m00);
            Point blackContoursCentreInFrame(centre.x+mainROI.x,centre.y+mainROI.y);
            Rect approxRect=boundingRect(blackContours[blackContoursMaxAreaI]);
            // to re-recongnise the centre of the contour to plot
            int blackROIXBegin,blackROIXRemain;
            int blackROIYBegin,blackROIYRemain;
            if(approxRect.x>=0){
                blackROIXBegin=approxRect.x;
            }
            else{
                blackROIXBegin=0;
            }
            if(blackROIXBegin+approxRect.width<frame.cols){
                blackROIXRemain=approxRect.width;
            }
            else{
                blackROIXRemain=frame.cols-blackROIXBegin;
            }
            if(blackContoursCentreInFrame.y-10>=0){
                blackROIYBegin=blackContoursCentreInFrame.y;
            }
            else{
                blackROIYBegin=0;
            }
            if(blackROIYBegin+20<=frame.rows){
                blackROIYRemain=20;
            }
            else{
                blackROIYRemain=frame.rows-blackROIYBegin;
            }
            Rect blackROI(blackROIXBegin,blackROIYBegin,blackROIXRemain,blackROIYRemain);   // blackRecognition range
            Mat reRecognition=hsv(blackROI);
            Scalar reLowerBlack=lowerBlack;//Scalar(0,0,0);
            Scalar reUpperBlack=upperBlack;//Scalar(180,255,30);      // MAX (180,255,255)
            Mat reMask;
            inRange(reRecognition,reLowerBlack,reUpperBlack,reMask);
            vector<vector<Point>> reContours;
            vector<Vec4i> reHierarchy;
            findContours(reMask,reContours,reHierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
            double reMaxArea=0;
            int reMaxAreaI=-1;
            for(int i=0;i<reContours.size();i++){
                double reArea=contourArea(reContours[i]);
                if(reArea>reMaxArea){
                    reMaxArea=reArea;
                    reMaxAreaI=i;
                }
            }
            if(reMaxArea!=-1){
                Moments reMu=moments(reContours[reMaxAreaI]);
                Point reCentre(reMu.m10/reMu.m00,reMu.m01/reMu.m00);
                Point reContoursCentreInFrame(reCentre.x+blackROI.x,reCentre.y+blackROI.y);
                distance=reContoursCentreInFrame.x-frameCentre.x;
                pidError=distance/10.0;
                if(display){
                    rectangle(frame,mainROI,Scalar(0,0,255),4*resizeScale);
                    drawMarker(frame,reContoursCentreInFrame,Scalar(0,0,255),MARKER_STAR,20*resizeScale,4*resizeScale,LINE_AA);
                    Point verA(reContoursCentreInFrame.x,0);
                    Point verB(reContoursCentreInFrame.x,frame.rows);
                    Point horA(0,reContoursCentreInFrame.y);
                    Point horB(frame.cols,reContoursCentreInFrame.y);
                    line(frame,verA,verB,Scalar(0,255,0),2*resizeScale,LINE_AA,0);
                    line(frame,horA,horB,Scalar(0,255,0),2*resizeScale,LINE_AA,0);

                    putText(frame,"Distance: "+to_string(distance),Point(20*resizeScale,80*resizeScale),FONT_HERSHEY_SIMPLEX,2*resizeScale,Scalar(0,0,255),4*resizeScale);

                    int tempBaseLine=0;
                    string coordinate="("+to_string(reContoursCentreInFrame.x-frameCentre.x)+", "+to_string(frameCentre.y-reContoursCentreInFrame.y)+")";
                    Size textSize=getTextSize(coordinate,FONT_HERSHEY_SIMPLEX,1.6*resizeScale,4*resizeScale,&tempBaseLine);
                    Point textOrg;
                    if(reContoursCentreInFrame.x+textSize.width<=frame.cols){
                        textOrg=Point(reContoursCentreInFrame.x,reContoursCentreInFrame.y+textSize.height+12);
                    }
                    else{
                        textOrg=Point(frameCentre.x*2-textSize.width,reContoursCentreInFrame.y-10+textSize.height+12);
                    }
                    putText(frame,coordinate,textOrg,FONT_HERSHEY_SIMPLEX,1.6*resizeScale,Scalar(0, 0, 255),4*resizeScale);
                }
                if(millis()%200<50){
                    lcdClear(lcd);
                    lcdPosition(lcd,0,0);
                    lcdPrintf(lcd,"error: %f",pidError);
                }
			}
			else{
                // out of view.
                pidError=114.5;
                if(display){
                    putText(frame,"out of view",Point(20*resizeScale,80*resizeScale),FONT_HERSHEY_SIMPLEX,2*resizeScale,Scalar(0,0,255),4*resizeScale);
                }
                if(millis()%200<50){
                    lcdClear(lcd);
                    lcdPosition(lcd,0,0);
                    lcdPrintf(lcd,"out of view");
                }
			}
		}
		else{
            // out of view.
            pidError=114.5;
			if(display){
				putText(frame,"out of view",Point(20*resizeScale,80*resizeScale),FONT_HERSHEY_SIMPLEX,2*resizeScale,Scalar(0,0,255),4*resizeScale);
			}
			if(millis()%200<50){
                lcdClear(lcd);
                lcdPosition(lcd,0,0);
                lcdPrintf(lcd,"finish task");
            }
        }
		calPid();
        gogogo();
		if(display){
			imshow("test",frame);
			waitKey(1);
		}
	}
	cap.release();
	destroyAllWindows();
}

void calPid(){
    P=pidError;
    I=pidError+lastI;
    D=pidError-lastError;
    pidValue=(Kp*P)+(Ki*I)+(Kd*D);
    lastError=pidError;
    lastI=I;
}

void gogogo(){
    if(pidError==114.5){
        sprintf(instruction,"#Barrrr %03d %03d %03d %03d",MOTOR_SPEED,MOTOR_SPEED,MOTOR_SPEED,MOTOR_SPEED);
		//sprintf(instruction,"#Barrrr %03d %03d %03d %03d",rightSpeed,leftSpeed,rightSpeed,leftSpeed);
        serialPuts(robot,instruction);
        delay(50);
        return;
    }
    leftSpeed=MOTOR_SPEED+pidValue;
    rightSpeed=MOTOR_SPEED-pidValue;
    if(leftSpeed<=70&&leftSpeed>=-70){
        //fine
    }
    else if(leftSpeed<-70){
        leftSpeed=-70;
    }
    else{
        leftSpeed=70;
    }
    if(rightSpeed<=60&&rightSpeed>=-70){
        //fine
    }
    else if(rightSpeed<-70){
        rightSpeed=-70;
    }
    else{
        rightSpeed=70;
    }
    if(abs(leftSpeed)+abs(rightSpeed)<60&&(abs(leftSpeed)<15)||abs(rightSpeed)<15){
        leftSpeed=(int)(leftSpeed*1.2);
        rightSpeed=(int)(rightSpeed*1.2);
    }
    if(leftSpeed>=0&&rightSpeed>=0){
        //sprintf(instruction,"#Brff %03d %03d",leftSpeed,rightSpeed);
        sprintf(instruction,"#Baffff %03d %03d %03d %03d",leftSpeed,rightSpeed,leftSpeed,rightSpeed);
    }
    else if(leftSpeed<0){
        //sprintf(instruction,"#Brrf %03d %03d",abs(leftSpeed),rightSpeed);
        sprintf(instruction,"#Barfrf %03d %03d %03d %03d",abs(leftSpeed),rightSpeed,abs(leftSpeed),rightSpeed);
    }
    else{
        //sprintf(instruction,"#Brfr %03d %03d",leftSpeed,abs(rightSpeed));
        sprintf(instruction,"#Bafrfr %03d %03d %03d %03d",leftSpeed,abs(rightSpeed),leftSpeed,abs(rightSpeed));
    }
    serialPuts(robot,instruction);
    //delay(120);
    if(millis()%200<50){
        lcdPosition(lcd,0,1);
        lcdPrintf(lcd,"%03d %03d",leftSpeed,rightSpeed);
    }
}

void stop(){
    serialPuts(robot,"#ha");
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"stop!");
}

void playMusic(){
    system("vlc renai.mp3 --play-and-exit");
}

void ledBlinking(){
	for(int i=0;i<5;i++){
		digitalWrite(LED_RED,HIGH);
		delay(500);
		digitalWrite(LED_RED,LOW);
		digitalWrite(LED_BLUE,HIGH);
		delay(500);
		digitalWrite(LED_BLUE,LOW);
	}
}

float getDistance(){
    struct timeval echoBegin,echoCheckPoint;
    long micros;
    float distance;
    digitalWrite(USS_TRIG,HIGH);
    delayMicroseconds(10);
    digitalWrite(USS_TRIG,LOW);
    long unsigned org=millis();
    while(digitalRead(USS_ECHO)==LOW){
        if(millis()-org>100){
            return 100;
        }
    }
    gettimeofday(&echoBegin,NULL);
    while(digitalRead(USS_ECHO)==HIGH);
    gettimeofday(&echoCheckPoint,NULL);
    micros=(echoCheckPoint.tv_sec-echoBegin.tv_sec)*1000000+echoCheckPoint.tv_usec-echoBegin.tv_usec;
    distance=(micros/2)/29.1;
    return distance;
}

void stopAtDistance(float distance){
    float dis;
    char dst[10];
    int errorCount=0;
    serialPuts(robot,"#Baffff 20 22 20 22");
    lcdClear(lcd);
    lcdPosition(lcd, 0, 0);
    lcdPrintf(lcd,"stop at 5cm");
    while(1) {
        dis=getDistance();
        printf("dis: %lf\n",dis);
        if(dis<=distance){
            errorCount++;
        }
        if(errorCount>1){
            serialPuts(robot,"#ha");
            break;
        }
        printf("%f\n",dis);
        sprintf(dst,"%f",dis);
        lcdPosition(lcd,0,1);
        lcdPrintf(lcd,"Dis: %s",dst);
    }
            serialPuts(robot,"#ha");
    lcdPosition(lcd,0,1);
    lcdPrintf(lcd, "stop!");
}

void shortCut(double mainROIPercent,int display,double resizeScale,int colour){ // 1.red(slow) 2.yellow(normal) 3.green(fast) 4.blue(split)
	cameraUp(0);
	Scalar lowerColour;
	Scalar upperColour;
	Scalar lowerRed1;
	Scalar upperRed1;
	if(colour==1){ // red
		MOTOR_SPEED=BASIC_SPEED-2;
		lowerColour=Scalar(170,50,50);
		upperColour=Scalar(180,255,255);
		lowerRed1=Scalar(0,255,255);
		upperRed1=Scalar(18,255,255);
	}
	else if(colour==2){ // yellow
		MOTOR_SPEED=BASIC_SPEED;
		lowerColour=Scalar(19,50,50);
		upperColour=Scalar(37,255,255);
	}
	else if(colour==3){ // green
		MOTOR_SPEED=BASIC_SPEED+2;
		lowerColour=Scalar(38,50,50);
		upperColour=Scalar(82,255,255);
	}
	else if(colour==4){ // blue
		MOTOR_SPEED==BASIC_SPEED;
		lowerColour=Scalar(83,50,50);
		upperColour=Scalar(127,255,255);
	}
	int distance=0;
	VideoCapture cap(0);
	if(!cap.isOpened()){
		cout<<"Error opening camera"<<endl;
		return;
	}
    int capWidth=(int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int capHeight=(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	Mat frame;
	Mat orgFrame;
	int flag=0;
	Point latestCentre(0,0);
	unsigned org=millis();
	while(1){
		cap>>orgFrame;
		resize(orgFrame,frame,Size(capWidth*resizeScale,capHeight*resizeScale));
		Mat hsv;
		cvtColor(frame,hsv,COLOR_BGR2HSV);
		Point frameCentre(frame.cols/2,frame.rows/2);
		Rect mainROI(0,(frame.rows-frame.rows*mainROIPercent)/2,frame.cols,frame.rows*mainROIPercent);
		Mat recognition=hsv(mainROI);
		Mat colourMask;
		if(colour==1){
			Mat mask1,mask2;
			inRange(recognition,lowerRed1,upperRed1,mask1);
			inRange(recognition,lowerColour,upperColour,mask2);
			colourMask=mask1+mask2;
		}
		else{
			inRange(recognition,lowerColour,upperColour,colourMask);
		}
		vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
		findContours(colourMask,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
		double maxArea1=0;
		double maxArea2=0;
		int maxArea1I=-1;
		int maxArea2I=-1;
		for(int i=0;i<contours.size();i++){
			double area=contourArea(contours[i]);
			if(area>maxArea1){
				maxArea2=maxArea1;
				maxArea2I=maxArea1I;
				maxArea1=area;
				maxArea1I=i;
			}
			else if(area>maxArea2){
				maxArea2=area;
				maxArea2I=i;
			}
		}
		if(maxArea1I!=-1){
            Point centre;
			if(colour==4){
				if(maxArea2I!=-1){ // blue,two_line
					flag++;
					Moments mu1=moments(contours[maxArea1I]);
					Moments mu2=moments(contours[maxArea2I]);
					Point centre1(mu1.m10/mu1.m00,mu1.m01/mu1.m00);
					Point centre2(mu2.m10/mu2.m00,mu2.m01/mu2.m00);
					centre=Point((centre1.x+centre2.x)/2,(centre1.y+centre2.y)/2);

				}
				else{ // blue,one_line
                    flag++;
                    Moments mu=moments(contours[maxArea1I]);
                    centre=Point(mu.m10/mu.m00,mu.m01/mu.m00);
					//flag++;
					//Moments mu=moments(contours[maxArea1I]);
					//centre=Point(mu.m10/mu.m00,mu.m01/mu.m00);
				}
			}
			else{ // red,yellow,green
				flag++;
				Moments mu=moments(contours[maxArea1I]);
				centre=Point(mu.m10/mu.m00,mu.m01/mu.m00);
			}
			Point contoursCentreInFrame(centre.x+mainROI.x,centre.y+mainROI.y);
			distance=contoursCentreInFrame.x-frameCentre.x;
			pidError=distance/10.0;
			if(display){
				rectangle(frame,mainROI,Scalar(0,0,255),4*resizeScale);
				drawMarker(frame,contoursCentreInFrame,Scalar(0,0,255),MARKER_STAR,20*resizeScale,4*resizeScale,LINE_AA);
				Point verA(contoursCentreInFrame.x,0);
				Point verB(contoursCentreInFrame.x,frame.rows);
				Point horA(0,contoursCentreInFrame.y);
				Point horB(frame.cols,contoursCentreInFrame.y);
				line(frame,verA,verB,Scalar(0,255,0),2*resizeScale,LINE_AA,0);
				line(frame,horA,horB,Scalar(0,255,0),2*resizeScale,LINE_AA,0);
				putText(frame,"Distance: "+to_string(distance),Point(20*resizeScale,80*resizeScale),FONT_HERSHEY_SIMPLEX,2*resizeScale,Scalar(0,0,255),4*resizeScale);
				int tempBaseLine=0;
				string coordinate="("+to_string(contoursCentreInFrame.x-frameCentre.x)+", "+to_string(frameCentre.y-contoursCentreInFrame.y)+")";
				Size textSize=getTextSize(coordinate,FONT_HERSHEY_SIMPLEX,1.6*resizeScale,4*resizeScale,&tempBaseLine);
				Point textOrg;
				if(contoursCentreInFrame.x+textSize.width<=frame.cols){
					textOrg=Point(contoursCentreInFrame.x,contoursCentreInFrame.y+textSize.height+12);
				}
				else{
					textOrg=Point(frameCentre.x*2-textSize.width,contoursCentreInFrame.y-10+textSize.height+12);
				}
				putText(frame,coordinate,textOrg,FONT_HERSHEY_SIMPLEX,1.6*resizeScale,Scalar(0, 0, 255),4*resizeScale);
				drawContours(frame,contours,maxArea1I,Scalar(255,255,0),2,LINE_8,hierarchy,0,Point(mainROI.x,mainROI.y));
				if(colour==4&&maxArea2I!=-1){
					drawContours(frame,contours,maxArea2I,Scalar(255,255,0),2,LINE_8,hierarchy,0,Point(mainROI.x,mainROI.y));
				}
			}
			if(millis()%200<50){
				lcdClear(lcd);
				lcdPosition(lcd,0,0);
				lcdPrintf(lcd,"error: %f",pidError);
				lcdPosition(lcd,0,1);
				if(colour==1){
					lcdPuts(lcd,"Red");
				}
				else if(colour==2){
					lcdPuts(lcd,"Yellow");
				}
				else if(colour==3){
					lcdPuts(lcd,"Green");
				}
				else if(colour==4){
					lcdPuts(lcd,"Blue");
				}
			}
            calPid();
            gogogo();
            if(display){
                imshow("test",frame);
                waitKey(1);
            }
		}
		else{ // out_of_view
            if(flag<1){
                pidError=0;
            }
			else{
                if(colour==3){
                    pidError=300;
                }
                else{
                    pidError=114.5;
                }
			}
			if(millis()-org>2500){
                MOTOR_SPEED=BASIC_SPEED;
                stop();
                if(colour==4){
                    serialPuts(robot,"#Bafrfr 050 060 050 060");
                    delay(1200);
                }
                else if(colour==2){
                    afterYellow=1;
                }
                serialPuts(robot,"#ha");
                cap.release();
                return;
			}
			if(millis()%200<50){
				lcdClear(lcd);
				lcdPosition(lcd,0,0);
				lcdPrintf(lcd,"out of view");
			}

            calPid();
            gogogo();
            if(display){
                imshow("test",frame);
                waitKey(1);
            }
		}
	}
}

void loadImg(string loadPath){
	Size targetSize(640,480);
    for (int i=1;i<=12;++i){
        string filename=loadPath+to_string(i)+".png";
        Mat img=imread(filename);
        if (!img.empty()){
            cvtColor(img,img,COLOR_BGR2GRAY);
            img.convertTo(img,CV_8U);
            resize(img,img,targetSize); // Resize img in-place
            importedImages.push_back(img);
        }
        else{
            cerr<<"Failed to load image: "<<filename<<endl;
        }
    }
}

int matchTemp()
{
	Size targetSize(640,480);
	system("raspistill -o KIRIN_PIC/pending_img.png");
    Mat pendingImg=imread("KIRIN_PIC/pending_img.png");
	Mat hsv,mask,binary;
	cvtColor(pendingImg,hsv,COLOR_BGR2HSV);
	Scalar lowerMagenta=Scalar(80,100,45);
    Scalar upperMagenta=Scalar(179,255,255);
	inRange(hsv,lowerMagenta,upperMagenta,mask);
	vector<vector<Point>> contours;
    findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
	int maxAreaI=-1;
	double maxArea=0;
	for(int i=0;i<contours.size();i++){
		double area=contourArea(contours[i]);
		if(area>maxArea){
			maxArea=area;
			maxAreaI=i;
		}
	}
	vector<Point> approx;
	if(maxAreaI!=-1){
		approxPolyDP(contours[maxAreaI],approx,arcLength(contours[maxAreaI],true)*0.02,true);
		Mat contourMask=Mat::zeros(pendingImg.size(),CV_8UC1);
		Mat result;
        pendingImg.copyTo(result, contourMask);
		findApprox(approx);
		Point2f srcVertices[4];
        Point2f dstVertices[4];
        for (int i=0;i<4;i++){
            srcVertices[i]=approx[i];
        }
		dstVertices[0]=Point2f(0,0);
        dstVertices[1]=Point2f(pendingImg.cols-1,0);
        dstVertices[2]=Point2f(pendingImg.cols-1,pendingImg.rows-1);
        dstVertices[3]=Point2f(0,pendingImg.rows-1);
		Mat perspectiveMatrix=getPerspectiveTransform(srcVertices,dstVertices);
        Mat dst;
        warpPerspective(pendingImg,dst,perspectiveMatrix,pendingImg.size());
		//waitKey(3);
        Mat hsvDst,maskDst,binaryDst;
        cvtColor(dst,hsvDst,COLOR_BGR2HSV);
        inRange(hsvDst,lowerMagenta,upperMagenta,maskDst);
        threshold(maskDst,binaryDst,0,255,THRESH_BINARY);
        Mat whiteMask=Mat::zeros(dst.size(),CV_8UC1);
        whiteMask.setTo(255,maskDst);
        //cvtColor(maskDst,maskDst,COLOR_BGR2HSV);
        maskDst.convertTo(maskDst,CV_8U);
        Mat maskFinal;
        resize(maskDst,maskFinal,targetSize);
        double maxMatch=0;
        Mat bestMatch;
        int bestMatchI=-1;
		for (int i=0;i<importedImages.size();i++){
            Mat result;
            matchTemplate(maskFinal,importedImages[i],result,TM_CCOEFF_NORMED);
            double minVal,maxVal;
            Point minLoc,maxLoc;
            minMaxLoc(result,&minVal,&maxVal,&minLoc,&maxLoc);
			if (maxVal>maxMatch){
                maxMatch=maxVal;
                bestMatch=importedImages[i];
                bestMatchI=i;
            }
        }
        if(maxMatch<0.38){
            cout<<"maxMatch"<<maxMatch<<endl;
            return -1;
        }
		if(bestMatchI!=-1&&bestMatchI!=3&&bestMatchI!=4&&bestMatchI!=5&&bestMatchI!=6){
            //cout<<"Most likely imported image: imported_image_"<<bestMatchI+1<<".jpg"<<endl;
			destroyAllWindows();
			return bestMatchI+1;
        }
		else if(bestMatchI<=6&&bestMatchI>=3){
            int j=-100;
            j=processImage(hsvDst);
            if(j!=0){
                bestMatch=importedImages[j];
                //cout<<"Most likely imported image: imported_image_"<<j+1<<".jpg"<<endl;
                destroyAllWindows();
                return j+1;
            }
			else{
				return -1;
			}
        }
	}
	else{
		return -1;
	}
}

void findApprox(vector<Point>&corners){
    vector<Point> top,bot;
    Point2f center(0,0);
    for(const auto& p:corners){
        center.x+=p.x;
        center.y+=p.y;
    }
    center.x/=corners.size();
    center.y/=corners.size();
    for(int i=0;i<corners.size();i++){
        if (corners[i].y < center.y){
            top.push_back(corners[i]);
		}
        else{
            bot.push_back(corners[i]);
		}
    }
    Point tl=top[0].x<top[1].x?top[0]:top[1];
    Point tr=top[0].x>top[1].x?top[0]:top[1];
    Point bl=bot[0].x<bot[1].x?bot[0]:bot[1];
    Point br=bot[0].x>bot[1].x?bot[0]:bot[1];
    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
}

int processImage(Mat& img){
	Size targetSize(640,480);
    resize(img,img,targetSize);
    Rect roi(390,260,160,160);
    Mat ROI=img(roi);
    //imshow("ROI",ROI);
    // 定义HSV中不同颜色的范围
    Scalar lowerYellow=Scalar(21,47,55);
	Scalar upperYellow=Scalar(54,255,255);
    Scalar lowerRed=Scalar(169,154,110);
	Scalar upperRed=Scalar(179,255,255);
    Scalar lowerGreen=Scalar(60,52,68);
	Scalar upperGreen=Scalar(104,199,255);
    Scalar lowerBlue=Scalar(89,138,99);
	Scalar upperBlue = Scalar(148, 255, 255);
    Mat maskBlue,maskGreen,maskRed,maskYellow;
    vector<int> colourCounts;
    inRange(ROI,lowerYellow,upperYellow,maskYellow);
    //imshow("Yellow",maskYellow);
    colourCounts.push_back(countNonZero(maskYellow)); // 黄色
    inRange(ROI,lowerRed,upperRed,maskRed);
    //imshow("Red",maskRed);
    colourCounts.push_back(countNonZero(maskRed)); // 红色
    inRange(ROI,lowerGreen,upperGreen,maskGreen);
    //imshow("Green",maskGreen);
    colourCounts.push_back(countNonZero(maskGreen)); // 绿色
    inRange(ROI,lowerBlue,upperBlue,maskBlue);
    //imshow("Blue",maskBlue);
    colourCounts.push_back(countNonZero(maskBlue)); // 蓝色
    // 代表不同颜色的数字
    const int Yellow=3,Red=4,Green=5,Blue=6,None=0;
    vector<int> colours={Yellow,Red,Green,Blue};
    // 找出像素数量最多的颜色的索引
    int maxI=distance(colourCounts.begin(),max_element(colourCounts.begin(),colourCounts.end()));
    // 返回对应的颜色数字，如果没有检测到颜色，返回0
    return colourCounts[maxI]>0?colours[maxI]:None;
}

int wait4Traffic(){
	Size targetSize(640,480);
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout<<"Error opening camera"<<endl;
        return -1;
    }
    int flag=0;
    Mat frame;
    while(1){
        cap>>frame;
        if(frame.empty()){
            break;
        }
		resize(frame,frame,targetSize);
        Mat hsv,mask,binary;
		cvtColor(frame,hsv,COLOR_BGR2HSV);
		Scalar lowerMagenta=Scalar(80,100,45);
        Scalar upperMagenta=Scalar(179,255,255);
		inRange(hsv,lowerMagenta,upperMagenta,mask);
        vector<vector<Point>> contours;
        findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        int maxAreaI=-1;
        double maxArea=0;
        for (int i=0;i<contours.size();i++){
            double area=contourArea(contours[i]);
            if(area>maxArea){
                maxArea=area;
                maxAreaI=i;
            }
        }
        vector<Point> approx;
        if (maxAreaI != -1){
            approxPolyDP(contours[maxAreaI],approx,arcLength(contours[maxAreaI],true)*0.02,true);
            drawContours(frame,vector<vector<Point>> {approx},-1,Scalar(0,0,0),2);
            // 创建轮廓掩模
            Mat contourMask=Mat::zeros(frame.size(),CV_8UC1);
            drawContours(contourMask,vector<vector<Point>> {approx},-1,Scalar(255),FILLED);
            // 保留轮廓内的内容
            Mat result;
            frame.copyTo(result,contourMask);
            findApprox(approx);
            int x=approx[1].x;
            int y=approx[1].y;
            Rect roi(x,y,150,200);
            Mat ROI=frame(roi);
            //imshow("ROI",ROI);
            Mat hsvROI,maskROI;
            cvtColor(ROI,hsvROI,COLOR_BGR2HSV);
            inRange(hsvROI,Scalar(52,80,80),Scalar(90,255,255),maskROI);
            //imshow("maskROI",maskROI);
            int greenLight=countNonZero(maskROI);
            if(greenLight>=80){
                flag++;
            }
            if(flag >= 10){
                cout<<"Green light!!!"<<endl;
				cap.release();
                return 0;
            }
        }
    }
    cap.release();
    destroyAllWindows();
    return 0;
}

void kickFootball()
{
	int flag=0;
    cameraUp (0.4);
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout<<"Error opening camera"<<endl;
    }
    Size targetSize ( 640, 480 );
    Mat frame0,frame;
    serialPuts(robot,"#Bafrfr 30 30 30 30");
    delay(700);
    while(1){
        cout<<"flag: "<<flag<<endl;
		cap>>frame0;
		if(frame0.empty()){
				break;
		}
		resize ( frame0, frame, targetSize );
		Mat hsv, mask;
		cvtColor ( frame, hsv, COLOR_BGR2HSV );
		inRange ( hsv, Scalar ( 20, 90, 61 ), Scalar ( 160, 255, 255), mask );
		vector<vector<Point>> contours;
		findContours ( mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
		if ( !contours.empty() && flag>=1){
			int maxContourIndex = -1;
			double maxContourArea = 0.0;

			for ( size_t i = 0; i < contours.size(); i++ ){
				double area = contourArea ( contours[i] );

				if ( area > maxContourArea && area >= 30){
					maxContourArea = area;
					maxContourIndex = i;
				}
			}

			if(maxContourIndex != -1){
				drawContours ( frame, contours, maxContourIndex, Scalar ( 0, 255, 0 ), 2 );
				Moments mu = moments ( contours[maxContourIndex] );
				Point centroid ( mu.m10 / mu.m00, mu.m01 / mu.m00 );
				circle ( frame, centroid, 5, Scalar ( 255, 0, 0 ), -1 ); // 绘制质心
				circle(frame, Point(320,240), 5, Scalar(0, 0, 255), -1); // 绘制画面中心

				if ( centroid.x < 310 || centroid.x > 330 )
				{
					cout<<"Continue to turn"<<endl;
					serialPuts(robot,"#Barfrf 30 35 30 35");

				}
				else if( centroid.x >= 310 && centroid.x <=330)
				{
					cout<<"Ball found"<<endl;
					serialPuts(robot,"#ha");
					delay(500);
					serialPuts(robot,"#Baffff 020 022 020 022");
					delay(2000);
					serialPuts(robot,"#ha");
                    serialPuts(robot,"#Barrrr 20 38 20 38");
                    delay(1700);
					serialPuts(robot,"#ha");
                    cameraUp(0);
					break;
				}
			}
		}
        else if(contours.empty()&&flag>=1){
            cout<<"Not found"<<endl;
            serialPuts(robot,"#Barfrf 030 035 030 035");
        }
		else if(!contours.empty()&&flag==0){
            serialPuts(robot,"#Baffff 020 022 020 022");
		}
		else{
            flag++;
		}
		imshow("test",frame);
		waitKey(1);
	}
    cap.release();
    destroyAllWindows();
    afterFootball=1;
}
