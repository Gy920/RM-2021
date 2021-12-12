#include "../include/imageprocess/imageprocess.h"

int main()
{
  Serial serial;
  serial.openPort();
  SolverParam solver_param;
  solver_param.readParam();
  int color = ENEMY_BLUE;
  ImageProcess process(0, 0, color, serial, solver_param);
  thread t1(&ImageProcess::image_producer,&process);
  thread t2(&ImageProcess::image_consumer,&process);
  // thread t3(&ImageProcess::send_data,&process);
  // thread t4(&ImageProcess::receive_data,&process);
  t1.join();
  t2.join();
  //  t3.join();
  //  t4.join();
  return 0;


  return 0;
}
