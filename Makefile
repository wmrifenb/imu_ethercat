all: stepthree

stepone:
	gcc -DHAVE_CONFIG_H -I. -I../..  -I../../include -Wall -g -O2 -MT ec_user_example-main.o -MD -MP -MF .deps/ec_user_example-main.Tpo -c -o ec_user_example-main.o `test -f 'main.c' || echo './'`main.c 

steptwo: stepone
	mv -f .deps/ec_user_example-main.Tpo .deps/ec_user_example-main.Po

stepthree: steptwo
	/bin/bash ../../libtool  --tag=CC   --mode=link gcc -I../../include -Wall -g -O2 -L../../lib/.libs -lethercat -o ec_user_example -lpthread ec_user_example-main.o IMUReader.c libftd2xx.a -ldl -lrt -lpthread 

clean:
	rm -f *.o; rm ec_user_example
