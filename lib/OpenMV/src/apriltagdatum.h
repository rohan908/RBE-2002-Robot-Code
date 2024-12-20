
#ifndef __APRILTAGDATUM_H
#define __APRILTAGDATUM_H

/*
 * AprilTagDatum is the data received from the camera 
*/
struct AprilTagDatum { 
    int16_t header,
    checksum, 
    id, 
    w,  
    x, 
    z, 
    rx, 
    nul; };

#endif
