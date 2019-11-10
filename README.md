# Raymarcher

A basic CPU raymarching program written in C that uses signed distance fields 
and produces bitmap images. Currently, the sdf functions are hard-coded but
this should be changed at some point.

To compile, run
```shell
gcc -o raymarch raymarch.c -lm -lpthread
```

