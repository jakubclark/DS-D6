void displaySandboxGame() {
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS))
    ;
  prevTime = t;

  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F, 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  int x = (int16_t)((res[1] << 8) | res[0]);
  int y = (int16_t)((res[3] << 8) | res[2]);
  int z = (int16_t)((res[5] << 8) | res[4]);

  float accelX = y;
  float accelY = -x;
  float accelZ = z;
  int16_t ax = -accelY / 256,  // Transform accelerometer axes
      ay = accelX / 256,       // to grain coordinate space
      az = abs(accelZ) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az; // Clip & invert
  ax -= az;                    // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1; // Range of random motion to add back in

  int32_t v2; // Velocity squared
  float v;    // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    v2 =
        (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) {      // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  uint16_t i, bytes, oldidx, newidx, delta;
  int16_t newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {  // If grain would go out of bounds
      newx = MAX_X;      // keep it inside, and
      grain[i].vx /= -2; // give a slight bounce off the wall
    } else if (newx < 0) {
      newx = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy / 256) * WIDTH + (newx / 256);             // New pixel #
    if ((oldidx != newidx) &&       // If grain is moving to a new pixel...
        img[newidx]) {              // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if (delta == 1) {             // 1 pixel left or right)
        newx = grain[i].x;          // Cancel X motion
        grain[i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx = oldidx;            // No pixel change
      } else if (delta == WIDTH) {  // 1 pixel up or down
        newy = grain[i].y;          // Cancel Y motion
        grain[i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx = oldidx;            // No pixel change
      } else {                      // Diagonal intersection is more tricky...
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) {  // That pixel's free!  Take it!  But...
            newy = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2; // and bounce Y velocity
          } else {             // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) {  // Pixel is free, take it, but first...
              newx = grain[i].x; // Cancel X motion
              grain[i].vx /= -2; // and bounce X velocity
            } else {             // Both spots are occupied
              newx = grain[i].x; // Cancel X & Y motion
              newy = grain[i].y;
              grain[i].vx /= -2; // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx = oldidx; // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) {  // Pixel's free!  Take it!  But...
            newx = grain[i].x; // Cancel X motion
            grain[i].vy /= -2; // and bounce X velocity
          } else {             // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) {  // Pixel is free, take it, but first...
              newy = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2; // and bounce Y velocity
            } else {             // Both spots are occupied
              newx = grain[i].x; // Cancel X & Y motion
              newy = grain[i].y;
              grain[i].vx /= -2; // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx = oldidx; // Not moving
            }
          }
        }
      }
    }
    grain[i].x = newx; // Update grain position
    grain[i].y = newy;
    img[oldidx] = 0;   // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255; // Set new spot
    grain[i].pos = newidx;
  }

  display.clearDisplay();
  for (i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos, yPos, WHITE);
  }
  display.display();
}
