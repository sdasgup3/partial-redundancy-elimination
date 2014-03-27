int func(int a, int b)
{
  int x = a*b;
  while(a > b) {
    if(x >= 0 ) {
      x = x + 2;
    } else {
      x = x / 2;
    }
    b ++;
  }

  return x;
}
