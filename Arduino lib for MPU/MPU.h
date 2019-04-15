/*
* MPU.c
*
* Created: 4/12/2019 4:07:32 PM
* Author : Ahmad Khaled
*/
class MPU
{
	public:
	MPU(int n); //mpu number 1 or 2
	virtual ~MPU();
	double getX();
	double getY();
	double getZ();
	int address;
	void setX(double x);
	void setY(double y);
	void setZ(double z);
	void calculateAngles();
	void setup();
	protected:
	private:
	double x,y,z;
};

