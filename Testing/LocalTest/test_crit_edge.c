int func(int a, int b)
{
  int x = a*b;
  if(a > b) {
    x = 0;
  } else {
    if(x >= 0 ) {
      x = x + 2;
    } 
  }

  return x;
}
