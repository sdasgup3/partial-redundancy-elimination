void func(int a, int b, int c)
{
  int x,y,z,  d;
    if(x >= 0 ) {
      y = a+b;
      a=c;
      x=a+b;
    } 

    switch (a) {
      case 0:
label_1:  
       y  =a+b;
        if(a>b) {
          x=c;
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
        if(a<b) {
          y = a+b;
          goto label_3;
        } 
        goto label_4;
        break;
    }

label_3:
          //z = a+b;
          a=c;
          goto label_4;

label_4:
          //x = a+b;
          goto label_end;

     label_end:     
    return;
}
