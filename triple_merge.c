#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <stdarg.h>

#define FRAME_SIZE 5822
#define DATA_SIZE 5760

char *myname;
int verbosity =0;

//__attribute__ ((format (printf, 2, 3)))
int
dp(int level, char *format, ...) {
	va_list ap;
	if (level > verbosity)
		return 0;
	va_start(ap, format);
	return vfprintf(stderr, format, ap);
}

void
usage(void) {
	fprintf(stderr, "Usage: %s image1 image2 image3\n", myname);
    exit(1);
}

int
main(int argc, char *argv[]) {
	int i,n,frame;
	unsigned char buffer[3][FRAME_SIZE];
	int fd[3], errors[3];;
	int uncorrected_errors = 0;
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
		errors[i] = 0;
	}
	for (frame = 0; ;frame++) {
		int interpolate_flags[3];
		for (i = 0; i < 3; i++)	{
			while (1) {
				if ((n = read(fd[i], buffer[i], FRAME_SIZE)) != FRAME_SIZE) {
					switch (n) {
					case -1:
						fprintf(stderr, "Read of '%s' failed ", argv[1+i]);
						perror("");
						exit(1);
					case 0:
						dp(0, "%s: %d uncorrectable errors\n", myname, uncorrected_errors);
						for (i = 0; i < 3; i++)
							dp(0, "%s: %d corrected errors in file %d\n", myname, errors[i], i);
						exit(0);
					default:
						dp(0, "Partial frame read from '%s'\n", argv[1+i]);
						dp(0, "%s: %d uncorrectable errors\n", myname, uncorrected_errors);
						for (i = 0; i < 3; i++)
							dp(0, "%s: %d corrected errors in file %d\n", myname, errors[i], i);
						exit(1);
					}
				}
				{
					unsigned char	*scode = buffer[i]+DATA_SIZE;
					unsigned char	*subid = scode+7*8;
					int pno1       = (subid[1] >> 4) & 0xf;
					int pno2       = (subid[2] >> 4) & 0xf;
					int pno3       = (subid[2] >> 0) & 0xf;
					int hex_pno    = (pno1 << 8) | (pno2 << 4) | (pno3 << 0);
					interpolate_flags[i]       = subid[3] & (0x40|0x20);
					if (verbosity > 2) {
						int sum = 0;
						for (n = 0; n < FRAME_SIZE; n++)
							sum += buffer[i][n];
						dp(3, "File %d: frame %d pno = %03x sum=%d\n", i, frame, hex_pno, sum);
					}
					if (frame != 0 || hex_pno != 0x0bb) 
						break;
					dp(2, "File %d: skipping frame because pno == 0x0bb\n", i);
				}
			}
		}

		for (n = 0; n < FRAME_SIZE; n++) {
			int value, n_values;
			if (buffer[0][n] == buffer[1][n] && buffer[1][n] == buffer[2][n])
				continue;
			n_values = 0;
			value = -1;
			for (i = 0; i < 3; i++) {
				if (!interpolate_flags[i] && buffer[i][n] != value) {
					n_values++;
					value = buffer[i][n];
				}
			}
			if (n_values == 1 && value != -1) {
				dp(2, "Frame %d byte %d fixing error based on interpolate flags (%02X %02X %02X) (%02X %02X %02X)\n", frame, n, buffer[0][n], buffer[1][n], buffer[2][n], interpolate_flags[0], interpolate_flags[1], interpolate_flags[2]);
				buffer[0][n] = value;
				for (i = 0; i < 3; i++)
					if (buffer[i][n] != value)
						errors[i]++;
			}
				
			if (buffer[0][n] == buffer[1][n]) {
				errors[2]++;
				dp(2, "Error in file 2 at frame %d byte %d (%02X %02X %02X)(%02X %02X %02X)\n", frame, n, buffer[0][n], buffer[1][n], buffer[2][n], interpolate_flags[0], interpolate_flags[1], interpolate_flags[2]);
			} else if (buffer[0][n] == buffer[2][n]) {
				errors[1]++;
				dp(2, "Error in file 1 at frame %d byte %d (%02X %02X %02X)(%02X %02X %02X)\n", frame, n, buffer[0][n], buffer[1][n], buffer[2][n], interpolate_flags[0], interpolate_flags[1], interpolate_flags[2]);
			} else if (buffer[1][n] != buffer[2][n]) {
				int choosing_file = 0;
				uncorrected_errors++;
				if (errors[0] <= errors[1]) {
					if (errors[0] > errors[2])
						choosing_file = 2;
				} else {
					if (errors[1] > errors[2])
						choosing_file = 2;
					else
						choosing_file = 1;
				}
				dp(1, "All files differ frame %d byte %d (%02X %02X %02X) using file %d (%02X %02X %02X)\n", frame, n, buffer[0][n], buffer[1][n], buffer[2][n], choosing_file, interpolate_flags[0], interpolate_flags[1], interpolate_flags[2]);
				buffer[0][n] = buffer[choosing_file][n];
			} else {
				errors[0]++;
				dp(2, "Error in file 0 at frame %d byte %d (%02X %02X %02X) (%02X %02X %02X)\n", frame, n, buffer[0][n], buffer[1][n], buffer[2][n], interpolate_flags[0], interpolate_flags[1], interpolate_flags[2]);
				buffer[0][n] = buffer[1][n];
			}
		}
		if ((n = write(1, buffer[0], FRAME_SIZE)) != FRAME_SIZE) {
			fprintf(stderr, "Write failed ");
			perror("");
			exit(1);
		}
		if (uncorrected_errors > FRAME_SIZE && uncorrected_errors > frame*FRAME_SIZE/16) {
			fprintf(stderr, "Stopping because %d uncorrected errors in %d frames\n", uncorrected_errors, frame);
			fprintf(stderr, "Tape image may be unaligned or badly damaged\n");
			for (i = 0; i < 3; i++)
				dp(0, "%s: %d corrected errors in file %d\n", myname, errors[i], i);
			exit(1);
		}
	}
}