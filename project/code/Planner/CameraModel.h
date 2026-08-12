#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

void CameraModel_MapPoint(float x, float y,
                          float roll, float pitch,
                          float *out_x, float *out_y);

void CameraModel_MapVector(float x, float y,
                           float vx, float vy,
                           float roll, float pitch,
                           float *out_vx, float *out_vy);

#endif /* CAMERA_MODEL_H */
