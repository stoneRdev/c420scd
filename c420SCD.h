
 
#ifndef _C420SCD_H
#define _C420SCD_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

#include <C420SCD_SERVO_LIMITS.h>

const uint8_t SERVO_SPEED = 50;
const uint8_t SERVO_ERROR_TOLERANCE = 50;
struct Joint {
	const int min;
	const int max;
	const int address;
	Adafruit_PWMServoDriver* driver;
	uint8_t id;
	int position;
	int toPosition;
	inline void step() {
		if(position < toPosition) {
			position += round((toPosition - position) * (SERVO_SPEED / 100));
			if(position > toPosition || (toPosition - position) * (SERVO_SPEED / 100) < (SERVO_ERROR_TOLERANCE / 100)) position = toPosition;
		} else if(position > toPosition) {
			position -= round((position - toPosition) * (SERVO_SPEED / 100));
			if(position < toPosition || (position - toPosition) * (SERVO_SPEED / 100) < (SERVO_ERROR_TOLERANCE / 100)) position = toPosition;
		}
		if(position == toPosition) id |= B10000000;
		else id &= B01111111;
		driver.setPWM(address,0,position);
	}
	inline uint8_t getPosition() {
		return toPosition;
	}
	inline uint8_t getActualPosition() {
		return position;
	}
	inline uint8_t getPercentPosition() {
		return round(((max - min) / (toPosition - min)) * 100);
	}
	inline uint8_t getActualPercentPosition() {
		return round(((max - min) / (position - min)) * 100);
	}
	inline void setPosition(int pos) {
		toPosition = min(max(min,pos),max);
	}
	inline void setPositionPercent(uint8_t pos) {
		pos <<= 3;
		pos >>= 3;
		toPosition = min(max(min,round((max - min) * (map(pos,0,32,0,100) / 100) + min)),max);
	}
	static inline Joint attach(uint8_t mask,Adafruit_PWMServoDriver* driver) {
		Joint joint;
		joint.driver = driver;
		joint.id = mask;
		if(mask & B00000001 && mask & B00000010) {joint.min = SERVO_FRH_MIN;joint.max = SERVO_FRH_MAX;joint.address = SERVO_FRH_ADDR;} 
		else if(mask & B00000001) {joint.min = SERVO_FLH_MIN;joint.max = SERVO_FLH_MAX;joint.address = SERVO_FLH_ADDR;} 
		else if(mask & B00000100 && mask & B00000010) {joint.min = SERVO_BLH_MIN;joint.max = SERVO_BLH_MAX;joint.address = SERVO_BLH_ADDR;} 
		else if(mask & B00000010) {joint.min = SERVO_BRH_MIN;joint.max = SERVO_BRH_MAX;joint.address = SERVO_BRH_ADDR;} 
		else if(mask & B00001000 && mask & B00000100) {joint.min = SERVO_FRF_MIN;joint.max = SERVO_FRF_MAX;joint.address = SERVO_FRF_ADDR;} 
		else if(mask & B00000100) {joint.min = SERVO_FLF_MIN;joint.max = SERVO_FLF_MAX;joint.address = SERVO_FLF_ADDR;} 
		else if(mask & B00010000 && mask & B00001000) {joint.min = SERVO_BLF_MIN;joint.max = SERVO_BLF_MAX;joint.address = SERVO_BLF_ADDR;} 
		else if(mask & B00001000) {joint.min = SERVO_BRF_MIN;joint.max = SERVO_BRF_MAX;joint.address = SERVO_BRF_ADDR;} 
		else if(mask & B00100000 && mask & B00010000) {joint.min = SERVO_FRT_MIN;joint.max = SERVO_FRT_MAX;joint.address = SERVO_FRT_ADDR;} 
		else if(mask & B00010000) {joint.min = SERVO_FLT_MIN;joint.max = SERVO_FLT_MAX;joint.address = SERVO_FLT_ADDR;} 
		else if(mask & B01000000 && mask & B00100000) {joint.min = SERVO_BRT_MIN;joint.max = SERVO_BRT_MAX;joint.address = SERVO_BRT_ADDR;} 
		else if(mask & B00100000) {joint.min = SERVO_BLT_MIN;joint.max = SERVO_BLT_MAX;joint.address = SERVO_BLT_ADDR;}
		if(joint.min && joint.max) {
			joint.toPosition = round((joint.max - joint.min) * .5 + joint.min);
			joint.position = toPosition;
		}
		return joint;
	}
}
struct Leg {
	const Joint tibia;
	const Joint femur;
	const Joint hip;
	const uint8_t id;
	uint8_t status = B00000000;
	inline void step() {
		if(!(status & B10000000)) return;
		if(status & B00010000) {
			hip.step();	
		}
		if(status & B00100000) {
			femur.step();	
		}
		if(status & B01000000) {
			tibia.step();	
		}
		if(tibia.id & B10000000) {status |= B00000001;} 
		else {status &= B11111110;}
		if(femur.id & B10000000) {status |= B00000010;} 
		else {status &= B11111101;}
		if(hip.id & B10000000) {status |= B00000100;} 
		else {status &= B11111011;}
		if(status & B00000001 && status & B00000010 && status & B00000100) status |= B00001000;
		else status &= B11110111;
	}
	inline Joint* getJoint(uint8_t address) {
		if(address & B00000001) {
			return tibia;
		}
		if(address & B00000010) {
			return femur;
		}
		if(address & B00000100) {
			return hip;
		}
	}
	inline uint8_t getPosition(uint8_t address) {
		if(address & B00000001) {
			return tibia.getPosition();
		}
		if(address & B00000010) {
			return femur.getPosition();
		}
		if(address & B00000100) {
			return hip.getPosition();
		}
	}
	inline uint8_t getActualPosition(uint8_t address) {
		if(address & B00000001) {
			return tibia.getActualPosition();
		}
		if(address & B00000010) {
			return femur.getActualPosition();
		}
		if(address & B00000100) {
			return hip.getActualPosition();
		}	
	}
	inline uint8_t getActualPercentPosition(uint8_t address) {
		if(address & B00000001) {
			return tibia.getActualPercentPosition();
		}
		if(address & B00000010) {
			return femur.getActualPercentPosition();
		}
		if(address & B00000100) {
			return hip.getActualPercentPosition();
		}	
	}
	inline uint8_t getPercentPosition(uint8_t address) {
		if(address & B00000001) {
			return tibia.getPercentPosition();
		}
		if(address & B00000010) {
			return femur.getPercentPosition();
		}
		if(address & B00000100) {
			return hip.getPercentPosition();
		}	
	}
	inline void adjust(uint8_t input) {
		uint8_t selectors = (input >> 5);
		uint8_t position = (input << 3) >> 3;
		if(selectors & B00000001) {
			hip.setPositionPercent(position);
		}
		if(selectors & B00000010) {
			femur.setPositionPercent(position);
		}
		if(selectors & B00000100) {
			tibia.setPositionPercent(position);
		}
	}
	inline void adjustActual(uint8_t selectors,int position) {
		if(selectors & B00000001) {
			hip.setPercent(position);
		}
		if(selectors & B00000010) {
			femur.setPercent(position);
		}
		if(selectors & B00000100) {
			tibia.setPercent(position);
		}
	}
	static inline Leg assign(uint8_t mask,Adafruit_PWMServoDriver* driver) {
		Leg leg;
		leg.id = mask;
		if(mask & B00000001 && mask & B00000010) {
			leg.tibia = Joint::attach(B00110000,driver);
			leg.femur = Joint::attach(B00001100,driver);
			leg.hip = Joint::attach(B00000011,driver);
		} 
		else if(mask & B00000001) {
			leg.tibia = Joint::attach(B00010000,driver);
			leg.femur = Joint::attach(B00000100,driver);
			leg.hip = Joint::attach(B00000001,driver);
		} 
		else if(mask & B00000100 && mask & B00000010) {
			leg.tibia = Joint::attach(B00100000,driver);
			leg.femur = Joint::attach(B00011000,driver);
			leg.hip = Joint::attach(B00000110,driver);
		} 
		else if(mask & B00000010) {
			leg.tibia = Joint::attach(B01100000,driver);
			leg.femur = Joint::attach(B00001000,driver);
			leg.hip = Joint::attach(B00000010,driver);	
		} 
		return leg;
	}
}
struct Base {
	const Leg frontLeft;
	const Leg frontRight;
	const Leg backLeft;
	const Leg backRight
	static Base* instance;
	uint8_t status;
	static inline Base* getInstance(Adafruit_PWMServoDriver* driver) {
		if(instance == 0) {
			instance = Base(driver);
		}
		return instance;
	}
	inline Leg* getLeg(uint8_t address) {
		if(address & B00010000) {
			return frontLeft;		
		}
		if(address & B00100000) {
			return frontRight;
		}
		if(address & B01000000) {
			return backRight;
		}
		if(address & B10000000) {
			return backLeft;
		}
	}
	inline Joint* getJoint(uint8_t address) {
		if(address & B00010000) {
			return frontLeft.getJoint(address);		
		}
		if(address & B00100000) {
			return frontRight.getJoint(address);
		}
		if(address & B01000000) {
			return backRight.getJoint(address);
		}
		if(address & B10000000) {
			return backLeft.getJoint(address);
		}
	}
	inline void step() {
		if(status & B00010000) {
			frontLeft.step();
		}
		if(status & B00100000) {
			frontRight.step();
		}
		if(status & B01000000) {
			backRight.step();
		}
		if(status & B10000000) {
			backLeft.step();
		}
		if(frontLeft.status & B00001000) {
			status &= B00001000;
		}
		if(frontRight.status & B00001000) {
			status &= B00000100;
		}
		if(backRight.status & B00001000) {
			status &= B00000010;
		}
		if(backLeft.status & B00001000) {
			status &= B00000001;
		}
	}
	inline void acceptInput(uint8_t address,uint8_t position) {
		position = (position << 3) >> 3;
		if(address & B00000001) {
			frontLeft.adjust(((address >> 4) << 5) | position );
		}
		if(address & B00000010) {
			frontRight.adjust(((address >> 4) << 5) | position );
		}
		if(address & B00000100) {
			backRight.adjust(((address >> 4) << 5) | position );
		}
		if(address & B00001000) {
			backLeft.adjust(((address >> 4) << 5) | position );
		}
	}
	inline void acceptActualInput(uint8_t address,int position) {
		if(address & B00000001) {
			frontLeft.adjustActual(address >> 4, position);
		}
		if(address & B00000010) {
			frontRight.adjust(address >> 4 , position );
		}
		if(address & B00000100) {
			backRight.adjust(address >> 4 , position );
		}
		if(address & B00001000) {
			backLeft.adjust(address >> 4 , position );
		}
	}
	inline uint8_t getPosition(uint8_t address) {
		if(address & B00010000) {
			return frontLeft.getPosition(address);
		}
		if(address & B00100000) {
			return frontRight.getPosition(address);
		}
		if(address & B01000000) {
			return backRight.getPosition(address);
		}
		if(address & B10000000) {
			return backLeft.getPosition(address);
		}
	}
	inline uint8_t getActualPosition(uint8_t address) {
		if(address & B00010000) {
			return frontLeft.getActualPosition(address);
		}
		if(address & B00100000) {
			return frontRight.getActualPosition(address);
		}
		if(address & B01000000) {
			return backRight.getActualPosition(address);
		}
		if(address & B10000000) {
			return backLeft.getActualPosition(address);
		}
	}
	inline uint8_t getPercentPosition(uint8_t address) {
		if(address & B00010000) {
			return frontLeft.getPercentPosition(address);
		}
		if(address & B00100000) {
			return frontRight.getPercentPosition(address);
		}
		if(address & B01000000) {
			return backRight.getPercentPosition(address);
		}
		if(address & B10000000) {
			return backLeft.getPercentPosition(address);
		}
	}
	inline uint8_t getActualPercentPosition(uint8_t address) {
		if(address & B00010000) {
			return frontLeft.getActualPercentPosition(address);
		}
		if(address & B00100000) {
			return frontRight.getActualPercentPosition(address);
		}
		if(address & B01000000) {
			return backRight.getActualPercentPosition(address);
		}
		if(address & B10000000) {
			return backLeft.getActualPercentPosition(address);
		}
	}
private:
	inline Base(Adafruit_PWMServoDriver* driver) {
		frontLeft = Leg::assign(B00000001,driver);
		frontRight = Leg::assign(B00000011,driver);
		backRight = Leg::assign(B00000010,driver);
		backLeft = Leg::assign(B00000110,driver);
	}
}

class C420SCD {
public:
	const Base base;
	static C420SCD* instance;
	Adafruit_PWMServoDriver* servoController;
	inline void init() {
		servoController.begin();
		servoController.setPWMFreq(60);
	}
	inline void acceptPositionAdjustment(uint16_t packet) {
		uint8_t address = (uint8_t)(packet >> 8);
		uint8_t position = (uint8_t)(packet);
		base.acceptInput(address,position);
	}
	inline void acceptActualPositionAdjustment(uint8_t address,int position) {
		base.acceptActualInput(address,position);
	}
	inline uint8_t getPosition(uint8_t address) {
		return base.getPosition(address);
	}
	inline uint8_t getActualPosition(uint8_t address) {
		return base.getActualPosition(address);
	}
	inline uint8_t getPercentPosition(uint8_t address) {
		return base.getPercentPosition(address);
	}
	inline uint8_t getActualPercentPosition(uint8_t address) {
		return base.getActualPercentPosition(address);
	}
	inline C420SCD* getInstance() {
		if(instance == 0) {
			instance = C420SCD();
		}
		return instance;
	}
private:
	inline C420SCD() {
		servoController = Adafruit_PWMServoDriver();
		base = Base();
	}
}

#endif _C420SCD_H