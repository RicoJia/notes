TARGET:rico_app
common_math.o:common_math/common_math.c
	gcc -c -I common_math common_math/common_math.c -o common_math/common_math.o
	#-I is to specify the root directory of header files. 
complex_math.o:complex_math/complex_math.c
	gcc -c -I complex_math complex_math/complex_math.c -o complex_math/complex_math.o
trig_math.o: trig_math/trig_math.c
	gcc -c -I trig_math trig_math/trig_math.c -o trig_math/trig_math.o
libcalc.a: common_math/common_math.o complex_math/complex_math.o trig_math/trig_math.o
	ar -rs libcalc.a common_math/common_math.o complex_math/complex_math.o trig_math/trig_math.o
main.o: main.c
	gcc -c -I common_math -I complex_math -I trig_math main.c -o main.o	#you need -I for every header file sub directory
rico_app: main.o libcalc.a
	gcc main.o -o rico_app -L . libcalc.a -lm
clean:
	rm common_math/common_math.o
	rm complex_math/complex_math.o
	rm trig_math/trig_math.o
	rm libcalc.a
	rm main.o
	rm rico_app



