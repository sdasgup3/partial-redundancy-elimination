void func(int a, int b, int c)
{
  int x,y,z,  d,w;
    if(x >= 0 ) {
      y = a+b;
      x=a+b;
    } 

    switch (a) {
      case 0:
label_1:  
        if(a>b) {
          y  =a+b;
          z = y + b;
          goto label_1;
        }
        goto label_3;
        break;
      case 1:
        goto label_end;
        break;
      case 2:
label_5:  
        if(a>b) {
          goto label_5;
        }
        if(a>b) {
          y = a+b;
          goto label_3;
        } 
        goto label_4;
        break;
    }

label_3:
          z = a+b;
          goto label_4;

label_4:
          x = a+b;
          z = x + b;
          goto label_end;

     label_end:     
    return;
}
