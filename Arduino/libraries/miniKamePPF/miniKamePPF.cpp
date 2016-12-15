#include "miniKamePPF.h"

int angToUsec(float value){
  return value/180 * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) + MIN_PULSE_WIDTH;
}

void miniKamePPF::init(){

  /*board_pins[0] = D1;
  board_pins[1] = D4,
  board_pins[2] = D8;
  board_pins[3] = D6;
  board_pins[4] = D7;
  board_pins[5] = D5;
  board_pins[6] = D2;
  board_pins[7] = D3;*/

  board_pins[0] = 10;
  board_pins[1] = 11,
  board_pins[2] = 2;
  board_pins[3] = 3;
  board_pins[4] = 4;
  board_pins[5] = 5;
  board_pins[6] = 6;
  board_pins[7] = 7;

  /*trim[0] = 0;
  trim[1] = -8;
  trim[2] = 8;
  trim[3] = 5;
  trim[4] = 2;
  trim[5] = -6;
  trim[6] = 6;
  trim[7] = 5;*/

  trim[0] = 0;
  trim[1] = 0;
  trim[2] = 0;
  trim[3] = 0;
  trim[4] = 0;
  trim[5] = 0;
  trim[6] = 0;
  trim[7] = 0;

  for (int i=0; i<8; i++) reverse[i] = 0;


  for(int i=0; i<8; i++){
    oscillator[i].start();
    servo[i].attach(board_pins[i]);
  }
  zero();
}

void miniKamePPF::turnR(float steps, float T=600){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 23;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  // int offset[] = {90+ap,90-ap,90-hi,90+hi,90-ap,90+ap,90+hi,90-hi}; ORIGINAL
  int offset[] = {base+ap,base-ap,base-hi,base+hi,base-ap,base+ap,base+hi,base-hi};
  int phase[] = {0,180,90,90,180,0,90,90};

  execute(steps, period, amplitude, offset, phase);
}

void miniKamePPF::turnL(float steps, float T=600){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 23;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  //  int offset[] = {90+ap,90-ap,90-hi,90+hi,90-ap,90+ap,90+hi,90-hi}; ORIGINAL
  int offset[] = {base+ap,base-ap,base-hi,base+hi,base-ap,base+ap,base+hi,base-hi};
  int phase[] = {180,0,90,90,0,180,90,90};

  execute(steps, period, amplitude, offset, phase);
}

void miniKamePPF::dance(float steps, float T=600){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 0;
  int z_amp = 40;
  int ap = 30;
  int hi = 20;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  //  int offset[] = {90+ap,90-ap,90-hi,90+hi,90-ap,90+ap,90+hi,90-hi}; ORIGINAL
  int offset[] = {base+ap,base-ap,base-hi,base+hi,base-ap,base+ap,base+hi,base-hi};
  int phase[] = {0,0,0,270,0,0,90,180};

  execute(steps, period, amplitude, offset, phase);
}

void miniKamePPF::frontBack(float steps, float T=600){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 30;
  int z_amp = 25;
  int ap = 20;
  int hi = 30;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  //  int offset[] = {90+ap,90-ap,90-hi,90+hi,90-ap,90+ap,90+hi,90-hi}; ORIGINAL
  int offset[] = {base+ap,base-ap,base-hi,base+hi,base-ap,base+ap,base+hi,base-hi};
  int phase[] = {0,180,270,90,0,180,90,270};

  execute(steps, period, amplitude, offset, phase);
}

void miniKamePPF::run(float steps, float T=5000){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 15;
  int front_x = 6;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  //  int offset[] = { 90+ap-front_x, 90-ap+front_x, 90-hi,90+hi,
  //    90-ap-front_x, 90+ap+front_x,     90+hi,    90-hi  };ORIGINAL
  int offset[] = {    base+ap-front_x,
    base-ap+front_x,
    base-hi,
    base+hi,
    base-ap-front_x,
    base+ap+front_x,
    base+hi,
    base-hi
  };

  int phase[] = {0,0,90,90,180,180,90,90};

  execute(steps, period, amplitude, offset, phase);
}

void miniKamePPF::omniWalk(float steps, float T, bool side, float turn_factor){
  int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 23;
  int front_x = 6 * (1-pow(turn_factor, 2));
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
  //  int offset[] = {    90+ap-front_x,    90-ap+front_x,    90-hi,    90+hi,
  //    90-ap-front_x,    90+ap+front_x,    90+hi,    90-hi  }; ORIGINAL
  int offset[] = {    base+ap-front_x,    base-ap+front_x,    base-hi,    base+hi,
    base-ap-front_x,    base+ap+front_x,    base+hi,    base-hi  };

    int phase[8];
    if (side){
      int phase1[] =  {0,   0,   90,  90,  180, 180, 90,  90};
      int phase2R[] = {0,   180, 90,  90,  180, 0,   90,  90};
      for (int i=0; i<8; i++)
      phase[i] = phase1[i]*(1-turn_factor) + phase2R[i]*turn_factor;
    }
    else{
      int phase1[] =  {0,   0,   90,  90,  180, 180, 90,  90};
      int phase2L[] = {180, 0,   90,  90,  0,   180, 90,  90};
      for (int i=0; i<8; i++)
      phase[i] = phase1[i]*(1-turn_factor) + phase2L[i]*turn_factor + oscillator[i].getPhaseProgress();
    }

    execute(steps, period, amplitude, offset, phase);
  }

  void miniKamePPF::moonwalkL(float steps, float T=5000){
    int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
    int z_amp = 45;
    float period[] = {T, T, T, T, T, T, T, T};
    int amplitude[] = {0,0,z_amp,z_amp,0,0,z_amp,z_amp};
    //  int offset[] = {90, 90, 90, 90, 90, 90, 90, 90}; ORIGINAL
    int offset[] = {base, base, base, base, base, base, base, base};
    int phase[] = {0,0,0,120,0,0,180,290};

    execute(steps, period, amplitude, offset, phase);
  }

  void miniKamePPF::walk(float steps, float T=5000){
    int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
    int x_amp = 15;
    int z_amp = 20;
    int ap = 20;
    int hi = 10;
    int front_x = 12;
    float period[] = {T, T, T/2, T/2, T, T, T/2, T/2};
    int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
    //  int offset[] = {   90+ap-front_x, 90-ap+front_x, 90-hi, 90+hi,
    //    90-ap-front_x, 90+ap+front_x, 90+hi,  90-hi }; ORIGINAL
    int offset[] = {   base+ap-front_x, base-ap+front_x, base-hi, base+hi,
      base-ap-front_x, base+ap+front_x, base+hi,  base-hi };

    int  phase[] = {90, 90, 270, 90, 270, 270, 90, 270};

      for (int i=0; i<8; i++){
        oscillator[i].reset();
        oscillator[i].setPeriod(period[i]);
        oscillator[i].setAmplitude(amplitude[i]);
        oscillator[i].setPhase(phase[i]);
        oscillator[i].setOffset(offset[i]);
      }

      _final_time = millis() + period[0]*steps;
      _init_time = millis();
      bool side;
      while (millis() < _final_time){
        side = (int)((millis()-_init_time) / (period[0]/2)) % 2;
        setServo(0, oscillator[0].refresh());
        setServo(1, oscillator[1].refresh());
        setServo(4, oscillator[4].refresh());
        setServo(5, oscillator[5].refresh());

        if (side == 0){
          setServo(3, oscillator[3].refresh());
          setServo(6, oscillator[6].refresh());
        }
        else{
          setServo(2, oscillator[2].refresh());
          setServo(7, oscillator[7].refresh());
        }
        delay(1);
      }
    }

    void miniKamePPF::upDown(float steps, float T=5000){
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
      int x_amp = 0;
      int z_amp = 35;
      int ap = 20;
      int hi = 25;
      int front_x = 0;
      float period[] = {T, T, T, T, T, T, T, T};
      int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp};
    //  int offset[] = {    90+ap-front_x, 90-ap+front_x, 90-hi, 90+hi,
    //    90-ap-front_x, 90+ap+front_x, 90+hi, 90-hi }; ORIGINAL
    int offset[] = {    base+ap-front_x, base-ap+front_x, base-hi, base+hi,
      base-ap-front_x, base+ap+front_x, base+hi, base-hi };
      int phase[] = {0,0,90,270,180,180,270,90};

      execute(steps, period, amplitude, offset, phase);
    }


    void miniKamePPF::pushUp(float steps, float T=600){
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
      int z_amp = 40;
      int x_amp = 65;
      int hi = 30;
      float period[] = {T, T, T, T, T, T, T, T};
      int amplitude[] = {0,0,z_amp,z_amp,0,0,0,0};
//  int offset[] = {90,90,90-hi,90+hi,90-x_amp,90+x_amp,90+hi,90-hi}; ORIGINAL
      int offset[] = {base,base,base-hi,base+hi,base-x_amp,base+x_amp,base+hi,base-hi};
      int phase[] = {0,0,0,180,0,0,0,180};

      execute(steps, period, amplitude, offset, phase);
    }

    void miniKamePPF::hello(){
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  //    float sentado[]={90+15,90-15,90-65,90+65,90+20,90-20,90+10,90-10}; ORIGINAL
      float sentado[]={base+15,base-15,base-65,base+65,base+20,base-20,base+10,base-10};
      moveServos(150, sentado);
      delay(200);

      int z_amp = 40;
      int x_amp = 60;
      int T=350;
      float period[] = {T, T, T, T, T, T, T, T};
      int amplitude[] = {0,50,0,50,0,0,0,0};
    //  int offset[] = {90+15,40,90-65,90,90+20,90-20,90+10,90-10}; ORIGINAL
      int offset[] = {base+15,40,base-65,base,base+20,base-20,base+10,base-10};
      int phase[] = {0,0,0,90,0,0,0,0};

      execute(4, period, amplitude, offset, phase);

  //    float goingUp[]={160,20,90,90,90-20,90+20,90+10,90-10}; ORIGINAL
  // PPF: OJO con la aplitud de base*2
      float goingUp[]={base*2,20,base,base,base-20,base+20,base+10,base-10};
      moveServos(500, goingUp);
      delay(200);

    }



    void miniKamePPF::jump(){
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
  //    float sentado[]={90+15,90-15,90-65,90+65,90+20,90-20,90+10,90-10}; ORIGINAL
      float sentado[]={base+15,base-15,base-65,base+65,base+20,base-20,base+10,base-10};
      int ap = 20;
      int hi = 35;
    //  float salto[] = {90+ap,90-ap,90-hi,90+hi,90-ap*3,90+ap*3,90+hi,90-hi}; ORIGINAL
      float salto[] = {base+ap,base-ap,base-hi,base+hi,base-ap*3,base+ap*3,base+hi,base-hi};
      moveServos(150, sentado);
      delay(200);
      moveServos(0, salto);
      delay(100);
      home();
    }

    void miniKamePPF::home(){
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
      int ap = 20; // amplitud horizontal, servos 0,1,4,5
      int hi = 35; // amplitud vertical, servos 2,3,6,7
//  int position[] = {90+ap,90-ap,90-hi,90+hi,90-ap,90+ap,90+hi,90-hi}; ORIGINAL
      int position[] = {base+ap,base-ap,base-hi,base+hi,base-ap,base+ap,base+hi,base-hi};

      for (int i=0; i<8; i++) setServo(i, position[i]);
    }

    void miniKamePPF::zero(){
      //    for (int i=0; i<8; i++) setServo(i, 90); // ORIGINAL
      int base= 65; // PPF: parametrizo posición base media de mis servos (pos "1")
      for (int i=0; i<8; i++) setServo(i, base); // PPF: modifico a punto medio mis servos
    }

    void miniKamePPF::reverseServo(int id){
      if (reverse[id])
      reverse[id] = 0;
      else
      reverse[id] = 1;
    }


    void miniKamePPF::setServo(int id, float target){
      if (!reverse[id])
      servo[id].writeMicroseconds(angToUsec(target+trim[id]));
      else
      servo[id].writeMicroseconds(angToUsec(180-(target+trim[id])));
      _servo_position[id] = target;
    }

    float miniKamePPF::getServo(int id){
      return _servo_position[id];
    }


    void miniKamePPF::moveServos(int time, float target[8]) {
      if (time>10){
        for (int i = 0; i < 8; i++)	_increment[i] = (target[i] - _servo_position[i]) / (time / 10.0);
        _final_time =  millis() + time;

        while (millis() < _final_time){
          _partial_time = millis() + 10;
          for (int i = 0; i < 8; i++) setServo(i, _servo_position[i] + _increment[i]);
          while (millis() < _partial_time); //pause
        }
      }
      else{
        for (int i = 0; i < 8; i++) setServo(i, target[i]);
      }
      for (int i = 0; i < 8; i++) _servo_position[i] = target[i];
    }

    void miniKamePPF::execute(float steps, float period[8], int amplitude[8], int offset[8], int phase[8]){

      for (int i=0; i<8; i++){
        oscillator[i].setPeriod(period[i]);
        oscillator[i].setAmplitude(amplitude[i]);
        oscillator[i].setPhase(phase[i]);
        oscillator[i].setOffset(offset[i]);
      }

      unsigned long global_time = millis();

      for (int i=0; i<8; i++) oscillator[i].setTime(global_time);

      _final_time = millis() + period[0]*steps;
      while (millis() < _final_time){
        for (int i=0; i<8; i++){
          setServo(i, oscillator[i].refresh());
        }
        yield();
      }
    }
