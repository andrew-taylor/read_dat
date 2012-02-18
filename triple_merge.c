#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

#define FRAME_SIZE 5822

char *myname;

void
usage(void) {
	fprintf(stderr, "Usage: %s image1 image2 image3\n", myname);
    exit(1);
}
int
main(int argc, char *argv[]) {
	int i,n,frame;
	unsigned char buffer[3][FRAME_SIZE];
	int fd[3];
	int errors = 0;
	myname = strrchr(argv[0], '/');
	if (myname == NULL)
		myname = argv[0];
	else
		myname++;
		
	if (argc != 4)
		usage();
	for (i = 0; i < 3; i++)	{
		if ((fd[i] = open(argv[1+i], O_RDONLY)) < 0) {
			fprintf(stderr, "Can not open argument '%s' ", argv[1+i]);
			perror("");
			exit(1);
		}
	}
	for (frame = 0; ;frame++) {
		for (i = 0; i < 3; i++)	{
			if ((n = read(fd[i], buffer[i], FRAME_SIZE)) != FRAME_SIZE) {
				switch (n) {
				case -1:
					fprintf(stderr, "Read of '%s' failed ", argv[1+i]);
					perror("");
					exit(1);
				case 0:
					fprintf(stderr, "%s: %d errors corrected\n", myname, errors);
					exit(0);
				default:
					fprintf(stderr, "Partial frame read from '%s'\n", argv[1+i]);
					fprintf(stderr, "%s: %d errors corrected\n", myname, errors);
					exit(1);
				}
			}
		}
		for (n = 0; n < FRAME_SIZE; n++) {
			if (buffer[0][n] == buffer[1][n] && buffer[1][n] == buffer[2][n])
				continue;
			errors++;
			if (buffer[0][n] == buffer[1][n]) {
//				fprintf(stderr, "Error in file 2 at frame %d byte %d\n", frame, n);
			} else if (buffer[0][n] == buffer[2][n]) {
//				fprintf(stderr, "Error in file 1 at frame %d byte %d\n", frame, n);
			} else if (buffer[1][n] != buffer[2][n]) {
				fprintf(stderr, "All files differ frame %d byte %d\n", frame, n);
			} else {
				buffer[0][n] = buffer[1][n];
//				fprintf(stderr, "Error in file 0 at frame %d byte %d\n", frame, n);
			}
		}
		if ((n = write(1, buffer[0], FRAME_SIZE)) != FRAME_SIZE) {
			fprintf(stderr, "Write failed ");
			perror("");
			exit(1);
		}
	}
}