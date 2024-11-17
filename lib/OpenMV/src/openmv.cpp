#include <openmv.h>

uint8_t OpenMV::getTagCount(void)
{
    uint8_t tagCount = 0;
    seesTag = false;

    //delayMicroseconds(100); // Give camera time to get ready

    /* Ask for one byte, which should hold the tag count
     * We should probably check to make sure we don't accidenally read a tag.
     * All tags start with 0xAA55, so if we get 0x55, then we should throw an error...someday...
     */
    if (Wire.requestFrom(CAMERA_I2C_ADDRESS, (uint8_t)1, true) == 1){
        tagCount = Wire.read();
        seesTag = true;
    }
    Serial.print(">tagCount:");
    Serial.println(tagCount);

    return tagCount;
}

bool OpenMV::readTag(AprilTagDatum& tag)
{
    bool retVal = false;

    //Wire.begin();
    //Wire.setClock(100000ul);

    /*
     * Here we have to give the camera a little time to get ready. It needs to load
     * any tag data into its queue. 100us is hit-and-miss; 200us is reliable; we use
     * 250us here for extra "safety". Tested at "full speed", the camera takes ~50ms
     * to detect a tag, so if you're not getting reliable tag reading, try upping this
     * value.
     */

    //delayMicroseconds(250); // Give camera a little time to get ready

    uint8_t buffer[sizeof(AprilTagDatum)];
    //use some explicit casts to suppress warnings
    if (Wire.requestFrom(CAMERA_I2C_ADDRESS, sizeof(AprilTagDatum), true) == sizeof(AprilTagDatum))
    {
        Serial.print("gotData");
      for(uint8_t i = 0; i < sizeof(AprilTagDatum); i++) buffer[i] = Wire.read();
      uint16_t checkSum = 0;
      for(uint8_t i = 4; i < sizeof(AprilTagDatum); i += 2) checkSum += ((buffer[i+1] & 0xFF) << 8) | ((buffer[i+0] & 0xFF) << 0);
      checkSum = checkSum & 0xFFFF;
      if (checkSum == (((buffer[3] & 0xFFFF) << 8) | ((buffer[2] & 0xFF) << 0))){
        Serial.print("checkSum success");
        memcpy(&tag, buffer, sizeof(AprilTagDatum));
        dataHandle(tag);
        retVal = true;
      }
    }

    return retVal;
}

void OpenMV::dataHandle(AprilTagDatum &tag){
    tag.rot = (float)(tag.rot) / 100;
    tag.x = -1 * (float)(tag.x) / 10;
    tag.y = -1 * (float)(tag.y) / 10;
    tag.z = -1 * (float)(tag.z) / 10;
    tag.rx = (float)(tag.rx) / 10;
    tag.ry = (float)(tag.ry) / 10;
    tag.rz = (float)(tag.rz) / 10;
}

bool OpenMV::checkUART(AprilTagDatum& tag)  
{
  bool retVal = false;
  while(Serial1.available())
  {
    uint8_t b = Serial1.read();
    if(handleUART(b))
    {
      memcpy(&tag, &mvArray, 16);
      retVal = true;
    }
  }

  return retVal;
}

bool OpenMV::handleUART(uint8_t b)
{
  bool retVal = false;
  switch(mvIndex)
  {
    case 0:
      if(b == 0xff) mvIndex++; //first byte must be 0xff
      break;
    case 1:
      if(b == 0x55) mvIndex++;
      else mvIndex = 0; //didn't get the 0x55 byte, so restart
      break;
    case 16:
      if(b == 0xaa) //correct end byte, so process
      {
        retVal = true;
        mvIndex = 0;
      } 
      else mvIndex = 0; //didn't get the aa byte, so restart
      break;
    case 17:
      Serial.println("Something is very wrong!"); //PANIC
      break;
    default:
      mvArray[mvIndex++] = b;
  }

  //TODO: add checksum verification

  return retVal;
}

float OpenMV::calcCenterError(AprilTagDatum& tag){
    float cx = tag.x - (tag.w / 2);
    return cx;
}

uint8_t OpenMV::PrintAprilTags()
{
    uint8_t tagCount = getTagCount();
    if(tagCount) 
    {
      //Serial.println(tagCount);
      if(readTag(tag)) // camera.readTag(tag))
      {
        Serial.print(F("id: "));
        Serial.print(tag.id);
        Serial.print(F(",[w: "));
        Serial.print(tag.w);
        Serial.print(F(", h: "));
        Serial.print(tag.h);
        Serial.print(F(", rot: "));
        Serial.print(tag.rot);
        Serial.print(F(", x: "));
        Serial.print(tag.x);
        Serial.print(F(", y: "));
        Serial.print(tag.y);
        Serial.print(F(", z: "));
        Serial.print(tag.z);
        Serial.print(F(", rx: "));
        Serial.print(tag.rx);
        Serial.print(F(", ry: "));
        Serial.print(tag.ry);
        Serial.print(F(", rz: "));
        Serial.print(tag.rz);
        Serial.println(F("]"));
        Serial.println(tag.checksum);
      }
    }

    return tagCount;
}