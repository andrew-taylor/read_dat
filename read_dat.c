/*

SYNOPSIS

read_dat [-m minimum_track_length] [-p filename-prefix] [-q] [-v verbosity-level] input-device-or-file

DESCRIPTION

read_dat reads data from a DAT in an audio-capable DDS drive
and produces a series of WAV files containing the audio data.  It also
produces a series of ".details" files containing information about each
".wav" file, including the date/time from the subcode data.

See http://www.cse.unsw.edu.au/~andrewt/read_dat/ for the latest version
of this program.

See http://www.btinternet.com/~big.bubbles/personal/ade/dat-dds/drives.html
and http://www.zianet.com/jgray/dat/index.html for more information about
audio-capable DDS drives.

OPTIONS

-a  --max_nonaudio_tape
	Maximum number of consecutive non-audio frames before program exits.
	Default is 10.
	
-A  --max_nonaudio_track
	Maximum number of consecutive non-audio frames before track closed
	Default is 0.
	
-d  --ignore_date_time
	Don't start a new track if the date/time jumps.
	
-m seconds  --minimum_track_length seconds
	Tracks less than this length will be ignored.  The value is a double.
	Default is 1.0 seconds.

-M seconds  --maximum_track_length seconds
	Do not create tracks longer than this number of seconds.  The value is a double.
	A new track will be started if this limit is reached. Default is 360000.0 seconds.

-n  --ignore_program_number
	Don't start a new track if the program number changes.

-p filename-prefix	--prefix filename-prefix
	Create WAV files named filename-prefix0.wav, filename-prefix1.wav ...
	Default is ""
	
-q	--quiet
	Turn off warnings.
	
-r seconds  --read_n_seconds seconds
	Read at most this number of seconds of audio.
	Default is 360000.0 seconds.

-s frames	--skip_n_frames
	Skip n frames on segment change.
	Default is 0.

-v verbosity-level	--verbose verbosity-level
	Print extra information.  The higher the level specified the more 
	information printed.  Verbosity-level should be in the range 1..5 
	
-V	--version
	Print the program version number

EXAMPLE

If /dev/st0 is an audio-capable DDS drive with a DAT inserted

read_dat /dev/st0

should create a series of ".wav" files containing the audio data
from the tape.  An accompany series of ".details" files will
also be created.

AUTHOR
	Andrew Taylor (andrewt@cse.unsw.edu.au)
	with additions by (Torsten Lang, read_dat@torstenlang.de (use read_dat in subject line))

DATE
    February 2001

COPYRIGHT

This software is copyright Andrew Taylor (andrewt@cse.unsw.edu.au).
It is released under the GNU General Public License. See the
terms and condions appended at end of this file.

BUGS

sometimes bad data at start of track encoded with 12-bit non-linear encoding (LP mode)

Untested on big-endian machines.

4-channel DATs untested.

Warning messages confusing.

Handling of bogus tracks of a small number of frames poor.

DISCUSSION

See ftp://ftp.informatik.uni-erlangen.de/pub/DATlib/ for a more
comprehensive library for manipulating DAT-tapes and 
http://www.hoxnet.com/dat-tools/datlib/ for a version modified for Linux.

No code from DATlib has been incorporated in this program but DATlib
provided invaluable information about the format of DAT tapes.

See http://www.zianet.com/jgray/dat/notes.html for some explanation of
the DAT data format.

I have not had access to a complete specification of the Audio DAT format.
This is what I've been able to infer:

Data on in an audio DAT tape is laid out in 5822 byte frames.

The first 5760 bytes of each frame are used for sound.

The last 62 bytes of the frame, i.e. frame[5760..5821] contain information
about the recording rather than sound.

The sound data on a DAT tape is either encoded as PCM 16-bit little-endian
samples or as 12-bit non-linear samples.

The follow applies only to 2 channel (stereo) tapes - I don't know
how what format is used if there are more channels.

There are three sampling rates used for 16-bit samples: 48khz, 44.1khz
and 32khz.

At 48khz frame[0..5759] contains are 1440 pairs of left & right 16-bit samples
At 44.1khz frame[0..5291] contains are 1323 pairs of left & right 16-bit samples
At 32khz frame[0..3840] contains are 960 pairs of left & right 16-bit samples

12-bit non-linear encoding is used with a 32khz sample-rate and half tape speed -
frame[0..5759] contains 1920 pairs of left & right 12-bit samples
I don't know how the non-linear encoding is done or how it is laid out
within the frame.

frame[5760..5815] is used for an array of 7 regions, each of 8 bytes.
The most significant 4 bits of the first byte in each  8 byte region
specifies the type of information it contains.

frame[5816..5819] contains the "sub-id".

frame[5820..5821] contains the "main-id".

A description of the encoding of the last 62 bytes should go here but
see the functions parse_frame, parse_subcodepack
for an operational description of  the format of the last 62 bytes.

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <getopt.h>
#include <utime.h>
#include <errno.h>


#define FRAME_SIZE 5822
#define DATA_SIZE 5760

#define SOUND_DATA_SIZE_48KHZ DATA_SIZE
#define SOUND_DATA_SIZE_44_1KHZ 5292
#define SOUND_DATA_SIZE_32KHZ_PCM 3840
#define SOUND_DATA_SIZE_32KHZ_NONLINEAR_PACKED DATA_SIZE
#define SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED 7680

#define PACKS_OFFSET 5760
#define N_PACKS 7
#define PACK_SIZE 8
#define SUBID_OFFSET (PACKS_OFFSET + (N_PACKS*PACK_SIZE))
#define MAINID_OFFSET (SUBID_OFFSET + 4)

#define MAX_FILENAME 8192
#define WAV_HEADER_LENGTH 44

#define CTRL_PRIO  8
#define CTRL_START 4
#define CTRL_SKIP  2
#define CTRL_TOC   1

typedef struct frame_info {
	int invalid;                // 0 == valid, 1 == invalid fields, 2 = non-audio
	int nChannels;
	int sampling_frequency;
	int encoding;
	int emphasis;
	time_t date_time;
	int program_number;
	int hex_pno;
	int interpolate_flags;
	int frame_number;
} frame_info_t;

void usage(void);
void process_file(char *filename);
void parse_frame(unsigned char *frame, frame_info_t *info);
int process_frame(unsigned char *frame, frame_info_t *previous_info, frame_info_t *info);
void parse_subcodepack(unsigned char *frame, int pack_index, frame_info_t *next_info);
void write_frame_audio(unsigned char *frame, frame_info_t *info);
void write_frame_nonlinear_audio(unsigned char *frame, frame_info_t *info);
void open_track(frame_info_t *info);
void close_track();
void write_track_details();
void warn(char *);
void die(char *s);
char *get_16bit_WAV_header(int samples, int channels, int frequency);
void intcpy(char *b, int i);
void shortcpy(char *b, int i);
int unBCD(unsigned int i);

char *decode_sampfreq[] = {"48kHz","44.1kHz","32kHz","reserved"};
char *decode_numchans[] = {"2 channels","4 channels","reserved","reserved"};
char *decode_quantization[] = {"16-bit linear","12-bit non-linear","reserved","reserved"};
char *decode_emphasis[] = {"none", "pre-emphasis"};
char *decode_subcodeid[] = {"Unused", "Program time","Absolute time","Running Time","Table of Contents","Date","Catalog","Catalog Number","International Standard Recording Code","Pro Binary"};
char *decode_weekday[] = {"Sun", "Mon","Tue","Wed","Thu","Fri","Sat"};
extern short decode_lp_sample[];
extern short translate_lp_frame_index[];
extern short decode_lp_sample[4096];

static int option_print_warnings = 1;
static int option_segment_on_datetime = 1;
static int option_segment_on_program_number = 1;

static int skip_frames_on_segment_change = 0;
static int verbosity = 1;
static off_t seek_n_frames = 0;
static double min_track_seconds = 1.0;
static double max_track_seconds = 360000.0;   /* 100 hours should be longer than any track or tape */
static double max_audio_seconds_read = 360000.0;
static int max_consecutive_nonaudio_frames_track = 0;
static int max_consecutive_nonaudio_frames_tape = 10;
static char *filename_prefix = "";
static char *myname;
static char *version = "0.7";
static int little_endian;

static int skip_n_frames = 0;
static double audio_seconds_read = 0;
static int consecutive_nonaudio_frames = 0;

static int track_number = 0;
static int track_frame_count = -1;
static int track_fd = -1;
char track_filename[MAX_FILENAME];
static int track_nSamples;
static int track_first_frame = -1;
static time_t track_first_date_time = -1;
static frame_info_t track_info;

static struct option long_options[] = {
	{"max_nonaudio_tape", 1, 0, 'a'},
	{"max_nonaudio_track", 1, 0, 'a'},
	{"ignore_date_time", 0, 0, 'd'},
	{"minimum_track_length", 1, 0, 'm'},
	{"maximum_track_length", 1, 0, 'M'},
	{"ignore_program_number", 0, 0, 'n'},
	{"prefix", 1, 0, 'p'},
	{"quiet", 0, 0, 'q'},
	{"read_n_seconds", 1, 0, 'r'},
	{"skip_n_frames", 1, 0, 's'},
	{"seek_n_frames", 1, 0, 'S'},
	{"verbose", 1, 0, 'v'},
	{"version", 0, 0, 'V'},
	{0, 0, 0, 0}
};

void
usage(void) {
	fprintf(stderr, "Usage: %s [-a frame_count] [-A frame_count] [-d] [-m minimum_track_length]  [-M maximum_track_length] [-n] [-p filename-prefix] [-r tape_seconds] [-s frames] [-S frames] [-q] [-v verbosity-level] input-device-or-file\n", myname);
    exit(1);
}

int
main(int argc, char *argv[]) {
	int n;

	myname = strrchr(argv[0], '/');
	if (myname == NULL)
		myname = argv[0];
	else
		myname++;
		
	n = 1;
	little_endian = (*(char *)&n == 1);
	if (!little_endian) {
		int i;
		for (i = 0; i < sizeof(decode_lp_sample)>>1; i++)
			decode_lp_sample[i] = (short)((unsigned char)(decode_lp_sample[i] >> 8) | (decode_lp_sample[i] << 8));
	}

	while (1) {
		int option_index;
		int c = getopt_long (argc, argv, "a:A:dm:M:np:qr:s:S:v:V", long_options, &option_index);
		if (c == -1)
			break;
		switch (c) {
		case 'a':
			max_consecutive_nonaudio_frames_tape = atoi(optarg);
			break;
		case 'A':
			max_consecutive_nonaudio_frames_track = atoi(optarg);
			if (max_consecutive_nonaudio_frames_tape < max_consecutive_nonaudio_frames_track)
				max_consecutive_nonaudio_frames_tape = max_consecutive_nonaudio_frames_track;
			break;
		case 'd':
			option_segment_on_datetime = 0;
			break;
		case 'm':
			min_track_seconds = atof(optarg);
			break;
		case 'M':
			max_track_seconds = atof(optarg);
			break;
		case 'n':
			option_segment_on_program_number = 0;
			break;
		case 'p':
			filename_prefix = optarg;
			break;
		case 'q':
			option_print_warnings = 0;
			verbosity = 0;
			break;
		case 'r':
			max_audio_seconds_read = atof(optarg);
			break;
		case 's':
			skip_frames_on_segment_change = atoi(optarg);
			if (skip_frames_on_segment_change < 0)
				usage();
			break;
		case 'S':
			seek_n_frames = atoi(optarg);
			if (seek_n_frames < 0)
				usage();
			break;
		case 'v':
			verbosity = atoi(optarg);
  			break;
		case 'V':
			printf("%s v%s - see http://www.cse.unsw.edu.au/~andrewt/read_dat/\n",myname, version);
  			break;
       	default:
        	usage();
		}
	}

	if (optind == argc)
		usage();
		
	for (;optind < argc;optind++) {
		process_file(argv[optind]);
	}
	return 0;
}

void
process_file(char *filename) {
	int fd, n;
	unsigned char buffer[FRAME_SIZE], next_buffer[FRAME_SIZE];
	frame_info_t info, next_info;
	int frame_number = 0;
	
	if ((fd = open(filename, O_RDONLY)) < 0)
		die("Can not open input");
	if (seek_n_frames) {
		if (verbosity > 0)
			printf("Seeking %d frames\n", (int)seek_n_frames);
		off_t seek_bytes = seek_n_frames*FRAME_SIZE;
		off_t seek_result = lseek(fd, seek_bytes, SEEK_SET);
		if (seek_result == seek_bytes) {
			if (verbosity > 1)
				printf("Seek succeeded\n");
			frame_number = seek_n_frames;
		} else if (seek_result <= 0) {
			if (verbosity > 0)
				printf("Seeking not possible reading %d frames\n", (int)seek_n_frames);
			for (;frame_number < seek_n_frames;frame_number++) {
				if (read(fd, buffer, FRAME_SIZE) != FRAME_SIZE)
					die("read failed");
			}
		} else
			die("can not recover from partial seek"); 
	}	
	if ((n = read(fd, buffer, FRAME_SIZE)) != FRAME_SIZE)
		die("read of first frame failed");
	info.frame_number = frame_number++;
	parse_frame(buffer, &info);
	for (;;frame_number++) {
		if ((n = read(fd, next_buffer, FRAME_SIZE)) != FRAME_SIZE) {
			switch (n) {
			case -1:
				die("read failed");
			case 0:
				process_frame(buffer, &info, &info);// hack to handle last frame
				close_track();
			default:
				die("read failed (short)");
			}
			break;
		}
		next_info.frame_number = frame_number;
		parse_frame(next_buffer, &next_info);
		if (!process_frame(buffer, &info, &next_info))
			return;
		memcpy(buffer, next_buffer, sizeof buffer);
		info = next_info;
	}
}


/*
 * process one frame (5822 bytes) of data,
 * return 0 if no more input should be read
 */
void
parse_frame(unsigned char *frame, frame_info_t *info) {
	unsigned char	*scode = frame+DATA_SIZE;
	unsigned char	*subid = scode+7*8;
	unsigned char	*mainid = subid+4;
	int channels   = (mainid[0] >> 0) & 0x3;
	int samplerate = (mainid[0] >> 2) & 0x3;
	int emphasis   = (mainid[0] >> 4) & 0x3;
	int fmtid      = (mainid[0] >> 6) & 0x3;
	int datapacket = (mainid[1] >> 0) & 0x3;
	int scms       = (mainid[1] >> 2) & 0x3;
	int width      = (mainid[1] >> 4) & 0x3;
	int encoding   = (mainid[1] >> 6) & 0x3;
	int dataid     = (subid[0] >> 0) & 0xf;
	int ctrlid     = (subid[0] >> 4) & 0xf;
	int numpacks   = (subid[1] >> 0) & 0xf;
	int pno1       = (subid[1] >> 4) & 0xf;
	int pno2       = (subid[2] >> 4) & 0xf;
	int pno3       = (subid[2] >> 0) & 0xf;
	int interpolate_flags       = subid[3];
	int hex_pno    = (pno1 << 8) | (pno2 << 4) | (pno3 << 0);
	int bcd_pno    = pno1 * 100 + pno2 * 10 + pno3 * 1;
	int pack_index;
	
	info->invalid = 0;
	info->program_number = -1;
	info->date_time = -1;
	info->nChannels = 2;
	info->sampling_frequency = 48000;
	info->hex_pno = hex_pno;
	info->interpolate_flags = interpolate_flags;
	
	if (dataid) {
		if (verbosity > 4)
			printf("Frame %d non audio dataid(%d)\n", info->frame_number, dataid);
		info->invalid = 2;
		return;
	}
	
	if ((ctrlid != 0 && verbosity >= 3) || verbosity >= 4)
		printf("Frame %d cntrlid=%d channels=%d samplerate=%d emphasis=%d fmtid=%d datapacket=%d scms=%d width=%d encoding=%d numpacks=%d id=%x pno=%x%x%x\n", info->frame_number, ctrlid, channels, samplerate, emphasis, fmtid, datapacket, scms, width, encoding, numpacks, subid[0], pno1,pno2, pno3);
	
	if (verbosity >= 4) {
		short *s;
		int i;
		printf("Frame %d data:", info->frame_number);
		s = (short *)&frame[0];
		for (i = 0; i < 10; i+=2)
			printf(" %4d", s[i]);
		printf(" ....");
		s = (short *)&frame[DATA_SIZE-60];
		for (i = 0; i < 10; i+=2)
			printf(" %4d", s[i]);
		printf("\n");
	}

	/* check for start id */
	if ((ctrlid & CTRL_START) && (ctrlid & CTRL_PRIO) && pno1 < 10 && pno2 < 10 && pno3 < 10)
		info->program_number = bcd_pno;
	for (pack_index = 0; pack_index < N_PACKS; pack_index++)
		parse_subcodepack(frame, pack_index, info);
	
	switch (channels) {
	case 0:
		info->nChannels = 2;
		break;
	case 1:
		info->nChannels = 4;
		break;
	default:
		info->invalid = 1;
		if (verbosity > 0)
			printf("Frame %d invalid value for channels(%d)\n", info->frame_number, channels);
	}
	
	switch (samplerate) {
	case 0:
		info->sampling_frequency = 48000;
		break;
	case 1:
		info->sampling_frequency = 44100;
		break;
	case 2:
		info->sampling_frequency = 32000;
		break;
	default:
		if (verbosity > 0)
			printf("Frame %d invalid value for sampling_frequency (%d)\n", info->frame_number, samplerate);
		info->invalid = 1;
	}
}

char *
frame_info_inconsistent(frame_info_t *i1, frame_info_t *i2) {
	if (option_segment_on_datetime && i1->date_time != -1 && i2->date_time != -1  && i1->date_time != i2->date_time  && i1->date_time != i2->date_time + 1 && i1->date_time != i2->date_time - 1)
		return "jump in subcode date/time";
	else if (i1->nChannels != i1->nChannels)
		return "change in number of channels";
	else if (i1->sampling_frequency != i2->sampling_frequency)
		return "change in sampling frequency";
	else if (option_segment_on_program_number  && i1->program_number != -1 && i2->program_number != -1  && i1->program_number != i2->program_number)
		return "change in program number";
	else if (i1->encoding != i2->encoding)
		return "change in encoding";
	else if (i1->emphasis != i2->emphasis)
		return "change in emphasis";
	else
		return NULL;
}

/*
 * process one frame (5822 bytes) of data,
 * return 0 if no more input should be read
 */
int
process_frame(unsigned char *frame, frame_info_t *info, frame_info_t *next_info) {
	if (info->hex_pno == 0x0ee) {
		if (verbosity >= 1)
			printf("Frame %d end of tape reached (0x0EE pno found)\n", info->frame_number);
		close_track();
		return 0;
	} else if (info->hex_pno == 0x0bb) {
		if (verbosity > 1)
			printf("Frame %d closing track 0x0BB pno seen\n", info->frame_number);
		if (track_fd == -1) {
			close_track();
			if (verbosity > 1)
				printf("Frame %d closing track 0x0BB pno seen\n", info->frame_number);
		}
		return 1;
	}
	if (info->interpolate_flags & (0x40|0x20)) {
		if (verbosity > 1)
			printf("Frame %d warning interpolate_flags set - ignoring\n", info->frame_number);
	}

	if (info->invalid != 2)
		consecutive_nonaudio_frames = 0;
	 else {
	 	if (consecutive_nonaudio_frames++ >= max_consecutive_nonaudio_frames_tape) {
			close_track();
			if (verbosity >= 1)
				printf("Exiting because because %d consecutive frames of non-audio data encountered\n", consecutive_nonaudio_frames);
			return 0;
		} else {
			if (track_fd == -1) {
				if (verbosity > 1)
					printf("Skipping frame %d because of non-audio dataid and not in track\n", info->frame_number);
				return 1;
			}
			if (next_info->invalid != 2 && !frame_info_inconsistent(&track_info, next_info)) {
				if (verbosity >= 1)
					printf("Frame %d ignoring non audio dataid  because next frame is audio and its frame info is with previous frame\n", info->frame_number);
			} else if (consecutive_nonaudio_frames >= max_consecutive_nonaudio_frames_track) {
				if (verbosity > 1)
					printf("Skipping frame %d because of non-audio dataid\n", info->frame_number);
				if (verbosity >= 1)
					printf("Closing track %d because %d frames of non-audio data encountered\n", track_number, consecutive_nonaudio_frames);
				close_track();
			} else {
				if (verbosity >= 1)
					printf("Ignoring non audio dataid on frame %d\n", info->frame_number);
			}
		}
		return 1;
	}

	if (track_fd != -1) {
		char *reason = frame_info_inconsistent(&track_info, info);
		if (reason != NULL && !frame_info_inconsistent(&track_info, next_info)) {
			if (verbosity >= 1)
				printf("Frame %d ignoring %s because previous & next frame consistent\n", info->frame_number, reason);
			info->nChannels = next_info->nChannels;
			info->sampling_frequency = next_info->sampling_frequency;
			info->encoding = next_info->encoding;
			info->emphasis = next_info->emphasis;
			info->program_number = next_info->program_number;
			info->date_time = next_info->date_time;
			reason = NULL;
		}
		if (reason != NULL) {
			close_track();
			skip_n_frames = skip_frames_on_segment_change;
			if (verbosity >= 2)
				printf("Closing track %d because %s\n", track_number, reason);
		}
	}
	if (skip_n_frames > 0) {
		skip_n_frames--;
		return 1;
	}
	
	if (track_fd == -1)
		open_track(info);
	if (track_first_frame == -1)
		track_first_frame = info->frame_number;
	track_info.frame_number = info->frame_number;
	if (info->date_time != -1) {
		track_info.date_time = info->date_time;
		if (track_first_date_time == -1)
			track_first_date_time = info->date_time;
	}
	if (info->program_number != -1 && track_info.program_number == -1)
		track_info.program_number = info->program_number;
	write_frame_audio(frame, info);
	if (audio_seconds_read >= max_audio_seconds_read) {
		if (verbosity >= 1)
			printf("Closing track %d and exiting, limit of %.2f seconds reached\n", track_number, max_audio_seconds_read);
		close_track();
		return 0;
	}
	if (track_nSamples/(double)info->sampling_frequency >= max_track_seconds) {
		if (verbosity >= 1)
			printf("Closing track %d and exiting, limit of %.2f seconds reached\n", track_number, max_track_seconds);
		close_track();
	}
	return 1;
}

/*
 * process one subcode pack (8 bytes of data)
 */
void
parse_subcodepack(unsigned char *frame, int pack_index, frame_info_t *info) {
	struct tm t;
	int j, weekday;
	int parity = 0;
	unsigned char *pack = frame + PACKS_OFFSET + pack_index * PACK_SIZE;
	int id = (pack[0] >> 4) & 0x0f;
	
	if (id == 0)
		return;
		
	for (j=0; j < 7; j++)
		parity ^= pack[j];
	if (parity != pack[7]) {
		if (verbosity >= 2)
			printf("Frame %d Subcode[%d] %s: Incorrect parity %x != %x\n", info->frame_number, pack_index, decode_subcodeid[id], parity, pack[7]);
		return;
	}

	switch (id) {
	case 1:
	case 2:
	case 3:
		if ((pack[3] != 0xAA && verbosity > 2) || verbosity > 3)
			printf("Frame %d Subcode[%d] %s: indexnr=%d %d:%d:%d frame=%d\n", info->frame_number, pack_index, decode_subcodeid[id], unBCD(pack[2]), unBCD(pack[3]), unBCD(pack[4]), unBCD(pack[5]), unBCD(pack[6]));
		break;
	case 5:
		weekday = pack[0]&0xf;
		if (weekday > 7) {
			if (verbosity >= 4)
				printf("Frame %d Subcode[%d] %s: invalid date\n", info->frame_number, pack_index, decode_subcodeid[id]);
			break;
		}
		t.tm_year = unBCD(pack[1]);
		if (t.tm_year < 50)
			t.tm_year += 100;
		t.tm_mon = unBCD(pack[2]) - 1;
		t.tm_mday = unBCD(pack[3]);
		t.tm_hour = unBCD(pack[4]) - 1;  /* maybe be incorrect - but seems necessary for my Sony TCD-D8 */
		t.tm_min = unBCD(pack[5]);
		t.tm_sec = unBCD(pack[6]);
		t.tm_wday = 0;
		t.tm_yday = 0; 
		t.tm_isdst = 0;
		if ((info->date_time = mktime(&t)) == (time_t)(-1)) {
			warn("can not convert time");
			break;
		}
		if (verbosity > 3)
			printf("Frame %d Subcode[%d] %s", info->frame_number, pack_index,ctime(&info->date_time));
		if (weekday - 1 != t.tm_wday)
			warn("Day of week apparently set incorrectly on recording  - using correct day of week");
		break;
	default:
		if (verbosity > 3)
			printf("Frame %d Subcode[%d] %s\n", info->frame_number, pack_index, decode_subcodeid[id]);
	}
}

/*
 * process the audio data from a signal frame
 */
void
write_frame_audio(unsigned char *frame, frame_info_t *info) {
	int n = 0;
	
	if (track_fd == -1)
		return;
	
	if (track_info.encoding != 0) {
		write_frame_nonlinear_audio(frame, info);
		return;
	}
	switch (track_info.sampling_frequency) {
	case 48000:
		n = SOUND_DATA_SIZE_48KHZ;
		break;
		
	case 44100:
		n = SOUND_DATA_SIZE_44_1KHZ;
		break;
		
	case 32000:
		n = SOUND_DATA_SIZE_32KHZ_PCM;
		break;
	
	default:
		die("internal error invalid track_sampling_frequency in write_frame_audio");
	}
	
	if (write(track_fd, frame, n) != n)
			die("write");
	track_nSamples += n / (2 * track_info.nChannels);
	audio_seconds_read += ((double)(n / (2 * track_info.nChannels)))/track_info.sampling_frequency;
	return;
}

void
write_frame_nonlinear_audio(unsigned char *frame, frame_info_t *info) {
	short buffer[SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED/2];
	int i, j;
	
	j = 0;
	for (i = 0; i < SOUND_DATA_SIZE_32KHZ_NONLINEAR_PACKED; i += 3) {
		int x0 = frame[translate_lp_frame_index[i]];
		int x1 = frame[translate_lp_frame_index[i+1]];
		int x2 = frame[translate_lp_frame_index[i+2]];
		buffer[j++] = decode_lp_sample[(x0 << 4) | ((x1 >> 4) & 0x0f)];
		buffer[j++] = decode_lp_sample[(x2 << 4) | (x1 & 0x0f)];
	}
	if (write(track_fd, buffer, SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED) != SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED)
			die("write");
	track_nSamples += SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED / (2 * track_info.nChannels);
	audio_seconds_read += ((double)(SOUND_DATA_SIZE_32KHZ_NONLINEAR_UNPACKED / (2 * track_info.nChannels)))/track_info.sampling_frequency;
}

void
create_filename(char *suffix, char *filename) {
	if (track_first_date_time > 0) {
		struct tm *t = localtime(&track_first_date_time);
		if (snprintf(filename, MAX_FILENAME, "%s%4d-%02d-%02d-%02d-%02d-%02d.%s", filename_prefix, t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, suffix) == -1)
			die("filename too long");
	} else {
		if (snprintf(filename, MAX_FILENAME, "%s%d.%s", filename_prefix, track_number, suffix) == -1)
			die("filename too long");
	}
}

/*
 * start a new track
 */
void
open_track(frame_info_t *info) {
	if (track_fd != -1)
		die("internal error open_track previous track not closed");
	track_nSamples = 0;
	track_info = *info;
	track_first_frame = info->frame_number;
	track_first_date_time = info->date_time;
	create_filename("wav", track_filename);
	if (verbosity >= 1)
		printf("Creating %s\n", track_filename);
	if ((track_fd = open(track_filename, O_CREAT|O_WRONLY|O_TRUNC, 0600)) < 0)
		die("Can not open file");
	/*
	 * header will be re-written when track is finished to add correct number of samples
	 */ 
	if (write(track_fd, get_16bit_WAV_header(track_nSamples, info->nChannels, info->sampling_frequency), WAV_HEADER_LENGTH) != WAV_HEADER_LENGTH)
		die("Can not write to file");
}

void
adjust_creation_time(char *filename) {
	struct utimbuf u;
	if (track_first_date_time > 0) {
		u.actime = track_first_date_time;
		u.modtime = track_first_date_time;
		utime(filename, &u);
	}
}	
/*
 * finish a track
 */
void
close_track() {
	char new_track_filename[MAX_FILENAME];
	double track_length = track_nSamples/(double)track_info.sampling_frequency;
	if (track_fd == -1)
		return;
		
	if (track_length < min_track_seconds) {
		if (verbosity >= 1) {
			if (track_nSamples == 0)
				printf("Deleting %s - no data\n", track_filename);
			else	
				printf("Deleting %s because %.2fs long - minimum track length %.2fs\n", track_filename, track_length, min_track_seconds);
		}
		/*
		 * This is inefficient but simpler than buffering writes
		 */
		close(track_fd);
		if (unlink(track_filename) < 0)
			die("unlink file");
	} else {
		if (lseek(track_fd, SEEK_SET, 0) < 0)
			die("Can not lseek track");
		if (verbosity >= 2)
			printf("Re-writing header to %s: %d channels of %d samples at %dhz\n", track_filename, track_info.nChannels, track_nSamples, track_info.sampling_frequency);
		/*
		 * Re-write header as we now know how many samples to include
		 */
		if (write(track_fd, get_16bit_WAV_header(track_nSamples, track_info.nChannels, track_info.sampling_frequency), WAV_HEADER_LENGTH) != WAV_HEADER_LENGTH)
			die("Can not write to file");
		close(track_fd);
		adjust_creation_time(track_filename);
		write_track_details();
		create_filename("wav", new_track_filename);
		if (strcmp(track_filename, new_track_filename) != 0) {
			if (verbosity > 0)
				printf("Renaming %s to %s\n", track_filename, new_track_filename);
			if (rename(track_filename, new_track_filename) != 0)
				die("can not rename track filename");
		}
		track_number++;
	}
	track_fd = -1;
	track_frame_count = -1;
	track_first_date_time = -1;
}

/*
 * create a ".details" file for a track
 */
void
write_track_details() {
	FILE *details_fp;
	char details_filename[MAX_FILENAME];
	create_filename("details", details_filename);
	if (verbosity >= 1)
		printf("Creating %s\n", details_filename);
	if ((details_fp = fopen(details_filename, "w")) == NULL)
		die("Can not open details file");
	fprintf(details_fp, "Sampling frequency: %d\n", track_info.sampling_frequency);
	fprintf(details_fp, "Channels: %d\n", track_info.nChannels);
	fprintf(details_fp, "Samples: %d\n", track_nSamples);
	fprintf(details_fp, "Quantization: %s\n", decode_quantization[track_info.encoding]);
	fprintf(details_fp, "Emphasis: %s\n", decode_emphasis[track_info.emphasis]);
	if (track_info.program_number <  0)
		fprintf(details_fp, "Program_number: --\n");
	else if (track_info.program_number != -1)
		fprintf(details_fp, "Program_number: %d\n", track_info.program_number);
	fprintf(details_fp, "First date: %s", ctime(&track_first_date_time));
	fprintf(details_fp, "Last date: %s", ctime(&(track_info.date_time)));
	fprintf(details_fp, "First frame: %d\n", track_first_frame);
	fprintf(details_fp, "Last frame: %d\n", track_info.frame_number);
	fclose(details_fp);
	adjust_creation_time(details_filename);
}	

/*
 * convert a BCD-encoded byte to decimal
 */
int
unBCD(unsigned int i) {
	return ((i >> 4) & 0x0f) * 10 + (i & 0x0f);
}


/*
 * create a suitable header for a WAV file
 */
char *
get_16bit_WAV_header(int samples, int channels, int frequency) {
	static char h[44];
	int bytesPerSample = 2;
	int bitsPerSample = 16;
	
	strcpy(h, "RIFF    WAVEfmt ");
	intcpy(h + 4, 36 + samples*channels*bytesPerSample);
	intcpy(h + 16, 16);
	shortcpy(h + 20, 1);
	shortcpy(h + 22, channels);
	intcpy(h + 24, frequency);
	intcpy(h + 28, frequency*channels*bytesPerSample);
	shortcpy(h + 32, bytesPerSample);
	shortcpy(h + 34, bitsPerSample);
	strcpy(h + 36, "data");
	intcpy(h + 40, samples*channels*bytesPerSample);
	return h;
}
	
void
intcpy(char *b, int i) {
	b[3] = (i >> 24);
	b[2] = ((i >> 16) & 0xff);
	b[1] = ((i >> 8) & 0xff);
	b[0] = (i & 0xff);
}

void
shortcpy(char *b, int i) {
	b[1] = ((i >> 8) & 0xff);
	b[0] = (i & 0xff);
}

void
die(char *s) {
	close_track();
	fprintf(stderr, "%s: ", myname);
	if (errno)
    	perror(s);
    else
		fprintf(stderr, "%s\n", s);
    exit(1);
}

void
warn(char *message) {
	static char last_track_number = -1;
	static char *last_message = "";
	
	if (!option_print_warnings)
		return;
	if (track_number == last_track_number && strcmp(message, last_message) == 0)
		return;
	printf("Warning: track %d: %s\n", track_number, message);
	last_track_number = track_number;
	last_message = message;
}


short decode_lp_sample[4096] = {
     0,      1,      2,      3,      4,      5,      6,      7,      8,      9,
    10,     11,     12,     13,     14,     15,     16,     17,     18,     19,
    20,     21,     22,     23,     24,     25,     26,     27,     28,     29,
    30,     31,     32,     33,     34,     35,     36,     37,     38,     39,
    40,     41,     42,     43,     44,     45,     46,     47,     48,     49,
    50,     51,     52,     53,     54,     55,     56,     57,     58,     59,
    60,     61,     62,     63,     64,     65,     66,     67,     68,     69,
    70,     71,     72,     73,     74,     75,     76,     77,     78,     79,
    80,     81,     82,     83,     84,     85,     86,     87,     88,     89,
    90,     91,     92,     93,     94,     95,     96,     97,     98,     99,
   100,    101,    102,    103,    104,    105,    106,    107,    108,    109,
   110,    111,    112,    113,    114,    115,    116,    117,    118,    119,
   120,    121,    122,    123,    124,    125,    126,    127,    128,    129,
   130,    131,    132,    133,    134,    135,    136,    137,    138,    139,
   140,    141,    142,    143,    144,    145,    146,    147,    148,    149,
   150,    151,    152,    153,    154,    155,    156,    157,    158,    159,
   160,    161,    162,    163,    164,    165,    166,    167,    168,    169,
   170,    171,    172,    173,    174,    175,    176,    177,    178,    179,
   180,    181,    182,    183,    184,    185,    186,    187,    188,    189,
   190,    191,    192,    193,    194,    195,    196,    197,    198,    199,
   200,    201,    202,    203,    204,    205,    206,    207,    208,    209,
   210,    211,    212,    213,    214,    215,    216,    217,    218,    219,
   220,    221,    222,    223,    224,    225,    226,    227,    228,    229,
   230,    231,    232,    233,    234,    235,    236,    237,    238,    239,
   240,    241,    242,    243,    244,    245,    246,    247,    248,    249,
   250,    251,    252,    253,    254,    255,    256,    257,    258,    259,
   260,    261,    262,    263,    264,    265,    266,    267,    268,    269,
   270,    271,    272,    273,    274,    275,    276,    277,    278,    279,
   280,    281,    282,    283,    284,    285,    286,    287,    288,    289,
   290,    291,    292,    293,    294,    295,    296,    297,    298,    299,
   300,    301,    302,    303,    304,    305,    306,    307,    308,    309,
   310,    311,    312,    313,    314,    315,    316,    317,    318,    319,
   320,    321,    322,    323,    324,    325,    326,    327,    328,    329,
   330,    331,    332,    333,    334,    335,    336,    337,    338,    339,
   340,    341,    342,    343,    344,    345,    346,    347,    348,    349,
   350,    351,    352,    353,    354,    355,    356,    357,    358,    359,
   360,    361,    362,    363,    364,    365,    366,    367,    368,    369,
   370,    371,    372,    373,    374,    375,    376,    377,    378,    379,
   380,    381,    382,    383,    384,    385,    386,    387,    388,    389,
   390,    391,    392,    393,    394,    395,    396,    397,    398,    399,
   400,    401,    402,    403,    404,    405,    406,    407,    408,    409,
   410,    411,    412,    413,    414,    415,    416,    417,    418,    419,
   420,    421,    422,    423,    424,    425,    426,    427,    428,    429,
   430,    431,    432,    433,    434,    435,    436,    437,    438,    439,
   440,    441,    442,    443,    444,    445,    446,    447,    448,    449,
   450,    451,    452,    453,    454,    455,    456,    457,    458,    459,
   460,    461,    462,    463,    464,    465,    466,    467,    468,    469,
   470,    471,    472,    473,    474,    475,    476,    477,    478,    479,
   480,    481,    482,    483,    484,    485,    486,    487,    488,    489,
   490,    491,    492,    493,    494,    495,    496,    497,    498,    499,
   500,    501,    502,    503,    504,    505,    506,    507,    508,    509,
   510,    511,    512,    514,    516,    518,    520,    522,    524,    526,
   528,    530,    532,    534,    536,    538,    540,    542,    544,    546,
   548,    550,    552,    554,    556,    558,    560,    562,    564,    566,
   568,    570,    572,    574,    576,    578,    580,    582,    584,    586,
   588,    590,    592,    594,    596,    598,    600,    602,    604,    606,
   608,    610,    612,    614,    616,    618,    620,    622,    624,    626,
   628,    630,    632,    634,    636,    638,    640,    642,    644,    646,
   648,    650,    652,    654,    656,    658,    660,    662,    664,    666,
   668,    670,    672,    674,    676,    678,    680,    682,    684,    686,
   688,    690,    692,    694,    696,    698,    700,    702,    704,    706,
   708,    710,    712,    714,    716,    718,    720,    722,    724,    726,
   728,    730,    732,    734,    736,    738,    740,    742,    744,    746,
   748,    750,    752,    754,    756,    758,    760,    762,    764,    766,
   768,    770,    772,    774,    776,    778,    780,    782,    784,    786,
   788,    790,    792,    794,    796,    798,    800,    802,    804,    806,
   808,    810,    812,    814,    816,    818,    820,    822,    824,    826,
   828,    830,    832,    834,    836,    838,    840,    842,    844,    846,
   848,    850,    852,    854,    856,    858,    860,    862,    864,    866,
   868,    870,    872,    874,    876,    878,    880,    882,    884,    886,
   888,    890,    892,    894,    896,    898,    900,    902,    904,    906,
   908,    910,    912,    914,    916,    918,    920,    922,    924,    926,
   928,    930,    932,    934,    936,    938,    940,    942,    944,    946,
   948,    950,    952,    954,    956,    958,    960,    962,    964,    966,
   968,    970,    972,    974,    976,    978,    980,    982,    984,    986,
   988,    990,    992,    994,    996,    998,   1000,   1002,   1004,   1006,
  1008,   1010,   1012,   1014,   1016,   1018,   1020,   1022,   1024,   1028,
  1032,   1036,   1040,   1044,   1048,   1052,   1056,   1060,   1064,   1068,
  1072,   1076,   1080,   1084,   1088,   1092,   1096,   1100,   1104,   1108,
  1112,   1116,   1120,   1124,   1128,   1132,   1136,   1140,   1144,   1148,
  1152,   1156,   1160,   1164,   1168,   1172,   1176,   1180,   1184,   1188,
  1192,   1196,   1200,   1204,   1208,   1212,   1216,   1220,   1224,   1228,
  1232,   1236,   1240,   1244,   1248,   1252,   1256,   1260,   1264,   1268,
  1272,   1276,   1280,   1284,   1288,   1292,   1296,   1300,   1304,   1308,
  1312,   1316,   1320,   1324,   1328,   1332,   1336,   1340,   1344,   1348,
  1352,   1356,   1360,   1364,   1368,   1372,   1376,   1380,   1384,   1388,
  1392,   1396,   1400,   1404,   1408,   1412,   1416,   1420,   1424,   1428,
  1432,   1436,   1440,   1444,   1448,   1452,   1456,   1460,   1464,   1468,
  1472,   1476,   1480,   1484,   1488,   1492,   1496,   1500,   1504,   1508,
  1512,   1516,   1520,   1524,   1528,   1532,   1536,   1540,   1544,   1548,
  1552,   1556,   1560,   1564,   1568,   1572,   1576,   1580,   1584,   1588,
  1592,   1596,   1600,   1604,   1608,   1612,   1616,   1620,   1624,   1628,
  1632,   1636,   1640,   1644,   1648,   1652,   1656,   1660,   1664,   1668,
  1672,   1676,   1680,   1684,   1688,   1692,   1696,   1700,   1704,   1708,
  1712,   1716,   1720,   1724,   1728,   1732,   1736,   1740,   1744,   1748,
  1752,   1756,   1760,   1764,   1768,   1772,   1776,   1780,   1784,   1788,
  1792,   1796,   1800,   1804,   1808,   1812,   1816,   1820,   1824,   1828,
  1832,   1836,   1840,   1844,   1848,   1852,   1856,   1860,   1864,   1868,
  1872,   1876,   1880,   1884,   1888,   1892,   1896,   1900,   1904,   1908,
  1912,   1916,   1920,   1924,   1928,   1932,   1936,   1940,   1944,   1948,
  1952,   1956,   1960,   1964,   1968,   1972,   1976,   1980,   1984,   1988,
  1992,   1996,   2000,   2004,   2008,   2012,   2016,   2020,   2024,   2028,
  2032,   2036,   2040,   2044,   2048,   2056,   2064,   2072,   2080,   2088,
  2096,   2104,   2112,   2120,   2128,   2136,   2144,   2152,   2160,   2168,
  2176,   2184,   2192,   2200,   2208,   2216,   2224,   2232,   2240,   2248,
  2256,   2264,   2272,   2280,   2288,   2296,   2304,   2312,   2320,   2328,
  2336,   2344,   2352,   2360,   2368,   2376,   2384,   2392,   2400,   2408,
  2416,   2424,   2432,   2440,   2448,   2456,   2464,   2472,   2480,   2488,
  2496,   2504,   2512,   2520,   2528,   2536,   2544,   2552,   2560,   2568,
  2576,   2584,   2592,   2600,   2608,   2616,   2624,   2632,   2640,   2648,
  2656,   2664,   2672,   2680,   2688,   2696,   2704,   2712,   2720,   2728,
  2736,   2744,   2752,   2760,   2768,   2776,   2784,   2792,   2800,   2808,
  2816,   2824,   2832,   2840,   2848,   2856,   2864,   2872,   2880,   2888,
  2896,   2904,   2912,   2920,   2928,   2936,   2944,   2952,   2960,   2968,
  2976,   2984,   2992,   3000,   3008,   3016,   3024,   3032,   3040,   3048,
  3056,   3064,   3072,   3080,   3088,   3096,   3104,   3112,   3120,   3128,
  3136,   3144,   3152,   3160,   3168,   3176,   3184,   3192,   3200,   3208,
  3216,   3224,   3232,   3240,   3248,   3256,   3264,   3272,   3280,   3288,
  3296,   3304,   3312,   3320,   3328,   3336,   3344,   3352,   3360,   3368,
  3376,   3384,   3392,   3400,   3408,   3416,   3424,   3432,   3440,   3448,
  3456,   3464,   3472,   3480,   3488,   3496,   3504,   3512,   3520,   3528,
  3536,   3544,   3552,   3560,   3568,   3576,   3584,   3592,   3600,   3608,
  3616,   3624,   3632,   3640,   3648,   3656,   3664,   3672,   3680,   3688,
  3696,   3704,   3712,   3720,   3728,   3736,   3744,   3752,   3760,   3768,
  3776,   3784,   3792,   3800,   3808,   3816,   3824,   3832,   3840,   3848,
  3856,   3864,   3872,   3880,   3888,   3896,   3904,   3912,   3920,   3928,
  3936,   3944,   3952,   3960,   3968,   3976,   3984,   3992,   4000,   4008,
  4016,   4024,   4032,   4040,   4048,   4056,   4064,   4072,   4080,   4088,
  4096,   4112,   4128,   4144,   4160,   4176,   4192,   4208,   4224,   4240,
  4256,   4272,   4288,   4304,   4320,   4336,   4352,   4368,   4384,   4400,
  4416,   4432,   4448,   4464,   4480,   4496,   4512,   4528,   4544,   4560,
  4576,   4592,   4608,   4624,   4640,   4656,   4672,   4688,   4704,   4720,
  4736,   4752,   4768,   4784,   4800,   4816,   4832,   4848,   4864,   4880,
  4896,   4912,   4928,   4944,   4960,   4976,   4992,   5008,   5024,   5040,
  5056,   5072,   5088,   5104,   5120,   5136,   5152,   5168,   5184,   5200,
  5216,   5232,   5248,   5264,   5280,   5296,   5312,   5328,   5344,   5360,
  5376,   5392,   5408,   5424,   5440,   5456,   5472,   5488,   5504,   5520,
  5536,   5552,   5568,   5584,   5600,   5616,   5632,   5648,   5664,   5680,
  5696,   5712,   5728,   5744,   5760,   5776,   5792,   5808,   5824,   5840,
  5856,   5872,   5888,   5904,   5920,   5936,   5952,   5968,   5984,   6000,
  6016,   6032,   6048,   6064,   6080,   6096,   6112,   6128,   6144,   6160,
  6176,   6192,   6208,   6224,   6240,   6256,   6272,   6288,   6304,   6320,
  6336,   6352,   6368,   6384,   6400,   6416,   6432,   6448,   6464,   6480,
  6496,   6512,   6528,   6544,   6560,   6576,   6592,   6608,   6624,   6640,
  6656,   6672,   6688,   6704,   6720,   6736,   6752,   6768,   6784,   6800,
  6816,   6832,   6848,   6864,   6880,   6896,   6912,   6928,   6944,   6960,
  6976,   6992,   7008,   7024,   7040,   7056,   7072,   7088,   7104,   7120,
  7136,   7152,   7168,   7184,   7200,   7216,   7232,   7248,   7264,   7280,
  7296,   7312,   7328,   7344,   7360,   7376,   7392,   7408,   7424,   7440,
  7456,   7472,   7488,   7504,   7520,   7536,   7552,   7568,   7584,   7600,
  7616,   7632,   7648,   7664,   7680,   7696,   7712,   7728,   7744,   7760,
  7776,   7792,   7808,   7824,   7840,   7856,   7872,   7888,   7904,   7920,
  7936,   7952,   7968,   7984,   8000,   8016,   8032,   8048,   8064,   8080,
  8096,   8112,   8128,   8144,   8160,   8176,   8192,   8224,   8256,   8288,
  8320,   8352,   8384,   8416,   8448,   8480,   8512,   8544,   8576,   8608,
  8640,   8672,   8704,   8736,   8768,   8800,   8832,   8864,   8896,   8928,
  8960,   8992,   9024,   9056,   9088,   9120,   9152,   9184,   9216,   9248,
  9280,   9312,   9344,   9376,   9408,   9440,   9472,   9504,   9536,   9568,
  9600,   9632,   9664,   9696,   9728,   9760,   9792,   9824,   9856,   9888,
  9920,   9952,   9984,  10016,  10048,  10080,  10112,  10144,  10176,  10208,
 10240,  10272,  10304,  10336,  10368,  10400,  10432,  10464,  10496,  10528,
 10560,  10592,  10624,  10656,  10688,  10720,  10752,  10784,  10816,  10848,
 10880,  10912,  10944,  10976,  11008,  11040,  11072,  11104,  11136,  11168,
 11200,  11232,  11264,  11296,  11328,  11360,  11392,  11424,  11456,  11488,
 11520,  11552,  11584,  11616,  11648,  11680,  11712,  11744,  11776,  11808,
 11840,  11872,  11904,  11936,  11968,  12000,  12032,  12064,  12096,  12128,
 12160,  12192,  12224,  12256,  12288,  12320,  12352,  12384,  12416,  12448,
 12480,  12512,  12544,  12576,  12608,  12640,  12672,  12704,  12736,  12768,
 12800,  12832,  12864,  12896,  12928,  12960,  12992,  13024,  13056,  13088,
 13120,  13152,  13184,  13216,  13248,  13280,  13312,  13344,  13376,  13408,
 13440,  13472,  13504,  13536,  13568,  13600,  13632,  13664,  13696,  13728,
 13760,  13792,  13824,  13856,  13888,  13920,  13952,  13984,  14016,  14048,
 14080,  14112,  14144,  14176,  14208,  14240,  14272,  14304,  14336,  14368,
 14400,  14432,  14464,  14496,  14528,  14560,  14592,  14624,  14656,  14688,
 14720,  14752,  14784,  14816,  14848,  14880,  14912,  14944,  14976,  15008,
 15040,  15072,  15104,  15136,  15168,  15200,  15232,  15264,  15296,  15328,
 15360,  15392,  15424,  15456,  15488,  15520,  15552,  15584,  15616,  15648,
 15680,  15712,  15744,  15776,  15808,  15840,  15872,  15904,  15936,  15968,
 16000,  16032,  16064,  16096,  16128,  16160,  16192,  16224,  16256,  16288,
 16320,  16352,  16384,  16448,  16512,  16576,  16640,  16704,  16768,  16832,
 16896,  16960,  17024,  17088,  17152,  17216,  17280,  17344,  17408,  17472,
 17536,  17600,  17664,  17728,  17792,  17856,  17920,  17984,  18048,  18112,
 18176,  18240,  18304,  18368,  18432,  18496,  18560,  18624,  18688,  18752,
 18816,  18880,  18944,  19008,  19072,  19136,  19200,  19264,  19328,  19392,
 19456,  19520,  19584,  19648,  19712,  19776,  19840,  19904,  19968,  20032,
 20096,  20160,  20224,  20288,  20352,  20416,  20480,  20544,  20608,  20672,
 20736,  20800,  20864,  20928,  20992,  21056,  21120,  21184,  21248,  21312,
 21376,  21440,  21504,  21568,  21632,  21696,  21760,  21824,  21888,  21952,
 22016,  22080,  22144,  22208,  22272,  22336,  22400,  22464,  22528,  22592,
 22656,  22720,  22784,  22848,  22912,  22976,  23040,  23104,  23168,  23232,
 23296,  23360,  23424,  23488,  23552,  23616,  23680,  23744,  23808,  23872,
 23936,  24000,  24064,  24128,  24192,  24256,  24320,  24384,  24448,  24512,
 24576,  24640,  24704,  24768,  24832,  24896,  24960,  25024,  25088,  25152,
 25216,  25280,  25344,  25408,  25472,  25536,  25600,  25664,  25728,  25792,
 25856,  25920,  25984,  26048,  26112,  26176,  26240,  26304,  26368,  26432,
 26496,  26560,  26624,  26688,  26752,  26816,  26880,  26944,  27008,  27072,
 27136,  27200,  27264,  27328,  27392,  27456,  27520,  27584,  27648,  27712,
 27776,  27840,  27904,  27968,  28032,  28096,  28160,  28224,  28288,  28352,
 28416,  28480,  28544,  28608,  28672,  28736,  28800,  28864,  28928,  28992,
 29056,  29120,  29184,  29248,  29312,  29376,  29440,  29504,  29568,  29632,
 29696,  29760,  29824,  29888,  29952,  30016,  30080,  30144,  30208,  30272,
 30336,  30400,  30464,  30528,  30592,  30656,  30720,  30784,  30848,  30912,
 30976,  31040,  31104,  31168,  31232,  31296,  31360,  31424,  31488,  31552,
 31616,  31680,  31744,  31808,  31872,  31936,  32000,  32064,  32128,  32192,
 32256,  32320,  32384,  32448,  32512,  32576,  32640,  32704, -32768, -32704,
-32640, -32576, -32512, -32448, -32384, -32320, -32256, -32192, -32128, -32064,
-32000, -31936, -31872, -31808, -31744, -31680, -31616, -31552, -31488, -31424,
-31360, -31296, -31232, -31168, -31104, -31040, -30976, -30912, -30848, -30784,
-30720, -30656, -30592, -30528, -30464, -30400, -30336, -30272, -30208, -30144,
-30080, -30016, -29952, -29888, -29824, -29760, -29696, -29632, -29568, -29504,
-29440, -29376, -29312, -29248, -29184, -29120, -29056, -28992, -28928, -28864,
-28800, -28736, -28672, -28608, -28544, -28480, -28416, -28352, -28288, -28224,
-28160, -28096, -28032, -27968, -27904, -27840, -27776, -27712, -27648, -27584,
-27520, -27456, -27392, -27328, -27264, -27200, -27136, -27072, -27008, -26944,
-26880, -26816, -26752, -26688, -26624, -26560, -26496, -26432, -26368, -26304,
-26240, -26176, -26112, -26048, -25984, -25920, -25856, -25792, -25728, -25664,
-25600, -25536, -25472, -25408, -25344, -25280, -25216, -25152, -25088, -25024,
-24960, -24896, -24832, -24768, -24704, -24640, -24576, -24512, -24448, -24384,
-24320, -24256, -24192, -24128, -24064, -24000, -23936, -23872, -23808, -23744,
-23680, -23616, -23552, -23488, -23424, -23360, -23296, -23232, -23168, -23104,
-23040, -22976, -22912, -22848, -22784, -22720, -22656, -22592, -22528, -22464,
-22400, -22336, -22272, -22208, -22144, -22080, -22016, -21952, -21888, -21824,
-21760, -21696, -21632, -21568, -21504, -21440, -21376, -21312, -21248, -21184,
-21120, -21056, -20992, -20928, -20864, -20800, -20736, -20672, -20608, -20544,
-20480, -20416, -20352, -20288, -20224, -20160, -20096, -20032, -19968, -19904,
-19840, -19776, -19712, -19648, -19584, -19520, -19456, -19392, -19328, -19264,
-19200, -19136, -19072, -19008, -18944, -18880, -18816, -18752, -18688, -18624,
-18560, -18496, -18432, -18368, -18304, -18240, -18176, -18112, -18048, -17984,
-17920, -17856, -17792, -17728, -17664, -17600, -17536, -17472, -17408, -17344,
-17280, -17216, -17152, -17088, -17024, -16960, -16896, -16832, -16768, -16704,
-16640, -16576, -16512, -16448, -16384, -16352, -16320, -16288, -16256, -16224,
-16192, -16160, -16128, -16096, -16064, -16032, -16000, -15968, -15936, -15904,
-15872, -15840, -15808, -15776, -15744, -15712, -15680, -15648, -15616, -15584,
-15552, -15520, -15488, -15456, -15424, -15392, -15360, -15328, -15296, -15264,
-15232, -15200, -15168, -15136, -15104, -15072, -15040, -15008, -14976, -14944,
-14912, -14880, -14848, -14816, -14784, -14752, -14720, -14688, -14656, -14624,
-14592, -14560, -14528, -14496, -14464, -14432, -14400, -14368, -14336, -14304,
-14272, -14240, -14208, -14176, -14144, -14112, -14080, -14048, -14016, -13984,
-13952, -13920, -13888, -13856, -13824, -13792, -13760, -13728, -13696, -13664,
-13632, -13600, -13568, -13536, -13504, -13472, -13440, -13408, -13376, -13344,
-13312, -13280, -13248, -13216, -13184, -13152, -13120, -13088, -13056, -13024,
-12992, -12960, -12928, -12896, -12864, -12832, -12800, -12768, -12736, -12704,
-12672, -12640, -12608, -12576, -12544, -12512, -12480, -12448, -12416, -12384,
-12352, -12320, -12288, -12256, -12224, -12192, -12160, -12128, -12096, -12064,
-12032, -12000, -11968, -11936, -11904, -11872, -11840, -11808, -11776, -11744,
-11712, -11680, -11648, -11616, -11584, -11552, -11520, -11488, -11456, -11424,
-11392, -11360, -11328, -11296, -11264, -11232, -11200, -11168, -11136, -11104,
-11072, -11040, -11008, -10976, -10944, -10912, -10880, -10848, -10816, -10784,
-10752, -10720, -10688, -10656, -10624, -10592, -10560, -10528, -10496, -10464,
-10432, -10400, -10368, -10336, -10304, -10272, -10240, -10208, -10176, -10144,
-10112, -10080, -10048, -10016,  -9984,  -9952,  -9920,  -9888,  -9856,  -9824,
 -9792,  -9760,  -9728,  -9696,  -9664,  -9632,  -9600,  -9568,  -9536,  -9504,
 -9472,  -9440,  -9408,  -9376,  -9344,  -9312,  -9280,  -9248,  -9216,  -9184,
 -9152,  -9120,  -9088,  -9056,  -9024,  -8992,  -8960,  -8928,  -8896,  -8864,
 -8832,  -8800,  -8768,  -8736,  -8704,  -8672,  -8640,  -8608,  -8576,  -8544,
 -8512,  -8480,  -8448,  -8416,  -8384,  -8352,  -8320,  -8288,  -8256,  -8224,
 -8192,  -8176,  -8160,  -8144,  -8128,  -8112,  -8096,  -8080,  -8064,  -8048,
 -8032,  -8016,  -8000,  -7984,  -7968,  -7952,  -7936,  -7920,  -7904,  -7888,
 -7872,  -7856,  -7840,  -7824,  -7808,  -7792,  -7776,  -7760,  -7744,  -7728,
 -7712,  -7696,  -7680,  -7664,  -7648,  -7632,  -7616,  -7600,  -7584,  -7568,
 -7552,  -7536,  -7520,  -7504,  -7488,  -7472,  -7456,  -7440,  -7424,  -7408,
 -7392,  -7376,  -7360,  -7344,  -7328,  -7312,  -7296,  -7280,  -7264,  -7248,
 -7232,  -7216,  -7200,  -7184,  -7168,  -7152,  -7136,  -7120,  -7104,  -7088,
 -7072,  -7056,  -7040,  -7024,  -7008,  -6992,  -6976,  -6960,  -6944,  -6928,
 -6912,  -6896,  -6880,  -6864,  -6848,  -6832,  -6816,  -6800,  -6784,  -6768,
 -6752,  -6736,  -6720,  -6704,  -6688,  -6672,  -6656,  -6640,  -6624,  -6608,
 -6592,  -6576,  -6560,  -6544,  -6528,  -6512,  -6496,  -6480,  -6464,  -6448,
 -6432,  -6416,  -6400,  -6384,  -6368,  -6352,  -6336,  -6320,  -6304,  -6288,
 -6272,  -6256,  -6240,  -6224,  -6208,  -6192,  -6176,  -6160,  -6144,  -6128,
 -6112,  -6096,  -6080,  -6064,  -6048,  -6032,  -6016,  -6000,  -5984,  -5968,
 -5952,  -5936,  -5920,  -5904,  -5888,  -5872,  -5856,  -5840,  -5824,  -5808,
 -5792,  -5776,  -5760,  -5744,  -5728,  -5712,  -5696,  -5680,  -5664,  -5648,
 -5632,  -5616,  -5600,  -5584,  -5568,  -5552,  -5536,  -5520,  -5504,  -5488,
 -5472,  -5456,  -5440,  -5424,  -5408,  -5392,  -5376,  -5360,  -5344,  -5328,
 -5312,  -5296,  -5280,  -5264,  -5248,  -5232,  -5216,  -5200,  -5184,  -5168,
 -5152,  -5136,  -5120,  -5104,  -5088,  -5072,  -5056,  -5040,  -5024,  -5008,
 -4992,  -4976,  -4960,  -4944,  -4928,  -4912,  -4896,  -4880,  -4864,  -4848,
 -4832,  -4816,  -4800,  -4784,  -4768,  -4752,  -4736,  -4720,  -4704,  -4688,
 -4672,  -4656,  -4640,  -4624,  -4608,  -4592,  -4576,  -4560,  -4544,  -4528,
 -4512,  -4496,  -4480,  -4464,  -4448,  -4432,  -4416,  -4400,  -4384,  -4368,
 -4352,  -4336,  -4320,  -4304,  -4288,  -4272,  -4256,  -4240,  -4224,  -4208,
 -4192,  -4176,  -4160,  -4144,  -4128,  -4112,  -4096,  -4088,  -4080,  -4072,
 -4064,  -4056,  -4048,  -4040,  -4032,  -4024,  -4016,  -4008,  -4000,  -3992,
 -3984,  -3976,  -3968,  -3960,  -3952,  -3944,  -3936,  -3928,  -3920,  -3912,
 -3904,  -3896,  -3888,  -3880,  -3872,  -3864,  -3856,  -3848,  -3840,  -3832,
 -3824,  -3816,  -3808,  -3800,  -3792,  -3784,  -3776,  -3768,  -3760,  -3752,
 -3744,  -3736,  -3728,  -3720,  -3712,  -3704,  -3696,  -3688,  -3680,  -3672,
 -3664,  -3656,  -3648,  -3640,  -3632,  -3624,  -3616,  -3608,  -3600,  -3592,
 -3584,  -3576,  -3568,  -3560,  -3552,  -3544,  -3536,  -3528,  -3520,  -3512,
 -3504,  -3496,  -3488,  -3480,  -3472,  -3464,  -3456,  -3448,  -3440,  -3432,
 -3424,  -3416,  -3408,  -3400,  -3392,  -3384,  -3376,  -3368,  -3360,  -3352,
 -3344,  -3336,  -3328,  -3320,  -3312,  -3304,  -3296,  -3288,  -3280,  -3272,
 -3264,  -3256,  -3248,  -3240,  -3232,  -3224,  -3216,  -3208,  -3200,  -3192,
 -3184,  -3176,  -3168,  -3160,  -3152,  -3144,  -3136,  -3128,  -3120,  -3112,
 -3104,  -3096,  -3088,  -3080,  -3072,  -3064,  -3056,  -3048,  -3040,  -3032,
 -3024,  -3016,  -3008,  -3000,  -2992,  -2984,  -2976,  -2968,  -2960,  -2952,
 -2944,  -2936,  -2928,  -2920,  -2912,  -2904,  -2896,  -2888,  -2880,  -2872,
 -2864,  -2856,  -2848,  -2840,  -2832,  -2824,  -2816,  -2808,  -2800,  -2792,
 -2784,  -2776,  -2768,  -2760,  -2752,  -2744,  -2736,  -2728,  -2720,  -2712,
 -2704,  -2696,  -2688,  -2680,  -2672,  -2664,  -2656,  -2648,  -2640,  -2632,
 -2624,  -2616,  -2608,  -2600,  -2592,  -2584,  -2576,  -2568,  -2560,  -2552,
 -2544,  -2536,  -2528,  -2520,  -2512,  -2504,  -2496,  -2488,  -2480,  -2472,
 -2464,  -2456,  -2448,  -2440,  -2432,  -2424,  -2416,  -2408,  -2400,  -2392,
 -2384,  -2376,  -2368,  -2360,  -2352,  -2344,  -2336,  -2328,  -2320,  -2312,
 -2304,  -2296,  -2288,  -2280,  -2272,  -2264,  -2256,  -2248,  -2240,  -2232,
 -2224,  -2216,  -2208,  -2200,  -2192,  -2184,  -2176,  -2168,  -2160,  -2152,
 -2144,  -2136,  -2128,  -2120,  -2112,  -2104,  -2096,  -2088,  -2080,  -2072,
 -2064,  -2056,  -2048,  -2044,  -2040,  -2036,  -2032,  -2028,  -2024,  -2020,
 -2016,  -2012,  -2008,  -2004,  -2000,  -1996,  -1992,  -1988,  -1984,  -1980,
 -1976,  -1972,  -1968,  -1964,  -1960,  -1956,  -1952,  -1948,  -1944,  -1940,
 -1936,  -1932,  -1928,  -1924,  -1920,  -1916,  -1912,  -1908,  -1904,  -1900,
 -1896,  -1892,  -1888,  -1884,  -1880,  -1876,  -1872,  -1868,  -1864,  -1860,
 -1856,  -1852,  -1848,  -1844,  -1840,  -1836,  -1832,  -1828,  -1824,  -1820,
 -1816,  -1812,  -1808,  -1804,  -1800,  -1796,  -1792,  -1788,  -1784,  -1780,
 -1776,  -1772,  -1768,  -1764,  -1760,  -1756,  -1752,  -1748,  -1744,  -1740,
 -1736,  -1732,  -1728,  -1724,  -1720,  -1716,  -1712,  -1708,  -1704,  -1700,
 -1696,  -1692,  -1688,  -1684,  -1680,  -1676,  -1672,  -1668,  -1664,  -1660,
 -1656,  -1652,  -1648,  -1644,  -1640,  -1636,  -1632,  -1628,  -1624,  -1620,
 -1616,  -1612,  -1608,  -1604,  -1600,  -1596,  -1592,  -1588,  -1584,  -1580,
 -1576,  -1572,  -1568,  -1564,  -1560,  -1556,  -1552,  -1548,  -1544,  -1540,
 -1536,  -1532,  -1528,  -1524,  -1520,  -1516,  -1512,  -1508,  -1504,  -1500,
 -1496,  -1492,  -1488,  -1484,  -1480,  -1476,  -1472,  -1468,  -1464,  -1460,
 -1456,  -1452,  -1448,  -1444,  -1440,  -1436,  -1432,  -1428,  -1424,  -1420,
 -1416,  -1412,  -1408,  -1404,  -1400,  -1396,  -1392,  -1388,  -1384,  -1380,
 -1376,  -1372,  -1368,  -1364,  -1360,  -1356,  -1352,  -1348,  -1344,  -1340,
 -1336,  -1332,  -1328,  -1324,  -1320,  -1316,  -1312,  -1308,  -1304,  -1300,
 -1296,  -1292,  -1288,  -1284,  -1280,  -1276,  -1272,  -1268,  -1264,  -1260,
 -1256,  -1252,  -1248,  -1244,  -1240,  -1236,  -1232,  -1228,  -1224,  -1220,
 -1216,  -1212,  -1208,  -1204,  -1200,  -1196,  -1192,  -1188,  -1184,  -1180,
 -1176,  -1172,  -1168,  -1164,  -1160,  -1156,  -1152,  -1148,  -1144,  -1140,
 -1136,  -1132,  -1128,  -1124,  -1120,  -1116,  -1112,  -1108,  -1104,  -1100,
 -1096,  -1092,  -1088,  -1084,  -1080,  -1076,  -1072,  -1068,  -1064,  -1060,
 -1056,  -1052,  -1048,  -1044,  -1040,  -1036,  -1032,  -1028,  -1024,  -1022,
 -1020,  -1018,  -1016,  -1014,  -1012,  -1010,  -1008,  -1006,  -1004,  -1002,
 -1000,   -998,   -996,   -994,   -992,   -990,   -988,   -986,   -984,   -982,
  -980,   -978,   -976,   -974,   -972,   -970,   -968,   -966,   -964,   -962,
  -960,   -958,   -956,   -954,   -952,   -950,   -948,   -946,   -944,   -942,
  -940,   -938,   -936,   -934,   -932,   -930,   -928,   -926,   -924,   -922,
  -920,   -918,   -916,   -914,   -912,   -910,   -908,   -906,   -904,   -902,
  -900,   -898,   -896,   -894,   -892,   -890,   -888,   -886,   -884,   -882,
  -880,   -878,   -876,   -874,   -872,   -870,   -868,   -866,   -864,   -862,
  -860,   -858,   -856,   -854,   -852,   -850,   -848,   -846,   -844,   -842,
  -840,   -838,   -836,   -834,   -832,   -830,   -828,   -826,   -824,   -822,
  -820,   -818,   -816,   -814,   -812,   -810,   -808,   -806,   -804,   -802,
  -800,   -798,   -796,   -794,   -792,   -790,   -788,   -786,   -784,   -782,
  -780,   -778,   -776,   -774,   -772,   -770,   -768,   -766,   -764,   -762,
  -760,   -758,   -756,   -754,   -752,   -750,   -748,   -746,   -744,   -742,
  -740,   -738,   -736,   -734,   -732,   -730,   -728,   -726,   -724,   -722,
  -720,   -718,   -716,   -714,   -712,   -710,   -708,   -706,   -704,   -702,
  -700,   -698,   -696,   -694,   -692,   -690,   -688,   -686,   -684,   -682,
  -680,   -678,   -676,   -674,   -672,   -670,   -668,   -666,   -664,   -662,
  -660,   -658,   -656,   -654,   -652,   -650,   -648,   -646,   -644,   -642,
  -640,   -638,   -636,   -634,   -632,   -630,   -628,   -626,   -624,   -622,
  -620,   -618,   -616,   -614,   -612,   -610,   -608,   -606,   -604,   -602,
  -600,   -598,   -596,   -594,   -592,   -590,   -588,   -586,   -584,   -582,
  -580,   -578,   -576,   -574,   -572,   -570,   -568,   -566,   -564,   -562,
  -560,   -558,   -556,   -554,   -552,   -550,   -548,   -546,   -544,   -542,
  -540,   -538,   -536,   -534,   -532,   -530,   -528,   -526,   -524,   -522,
  -520,   -518,   -516,   -514,   -512,   -511,   -510,   -509,   -508,   -507,
  -506,   -505,   -504,   -503,   -502,   -501,   -500,   -499,   -498,   -497,
  -496,   -495,   -494,   -493,   -492,   -491,   -490,   -489,   -488,   -487,
  -486,   -485,   -484,   -483,   -482,   -481,   -480,   -479,   -478,   -477,
  -476,   -475,   -474,   -473,   -472,   -471,   -470,   -469,   -468,   -467,
  -466,   -465,   -464,   -463,   -462,   -461,   -460,   -459,   -458,   -457,
  -456,   -455,   -454,   -453,   -452,   -451,   -450,   -449,   -448,   -447,
  -446,   -445,   -444,   -443,   -442,   -441,   -440,   -439,   -438,   -437,
  -436,   -435,   -434,   -433,   -432,   -431,   -430,   -429,   -428,   -427,
  -426,   -425,   -424,   -423,   -422,   -421,   -420,   -419,   -418,   -417,
  -416,   -415,   -414,   -413,   -412,   -411,   -410,   -409,   -408,   -407,
  -406,   -405,   -404,   -403,   -402,   -401,   -400,   -399,   -398,   -397,
  -396,   -395,   -394,   -393,   -392,   -391,   -390,   -389,   -388,   -387,
  -386,   -385,   -384,   -383,   -382,   -381,   -380,   -379,   -378,   -377,
  -376,   -375,   -374,   -373,   -372,   -371,   -370,   -369,   -368,   -367,
  -366,   -365,   -364,   -363,   -362,   -361,   -360,   -359,   -358,   -357,
  -356,   -355,   -354,   -353,   -352,   -351,   -350,   -349,   -348,   -347,
  -346,   -345,   -344,   -343,   -342,   -341,   -340,   -339,   -338,   -337,
  -336,   -335,   -334,   -333,   -332,   -331,   -330,   -329,   -328,   -327,
  -326,   -325,   -324,   -323,   -322,   -321,   -320,   -319,   -318,   -317,
  -316,   -315,   -314,   -313,   -312,   -311,   -310,   -309,   -308,   -307,
  -306,   -305,   -304,   -303,   -302,   -301,   -300,   -299,   -298,   -297,
  -296,   -295,   -294,   -293,   -292,   -291,   -290,   -289,   -288,   -287,
  -286,   -285,   -284,   -283,   -282,   -281,   -280,   -279,   -278,   -277,
  -276,   -275,   -274,   -273,   -272,   -271,   -270,   -269,   -268,   -267,
  -266,   -265,   -264,   -263,   -262,   -261,   -260,   -259,   -258,   -257,
  -256,   -255,   -254,   -253,   -252,   -251,   -250,   -249,   -248,   -247,
  -246,   -245,   -244,   -243,   -242,   -241,   -240,   -239,   -238,   -237,
  -236,   -235,   -234,   -233,   -232,   -231,   -230,   -229,   -228,   -227,
  -226,   -225,   -224,   -223,   -222,   -221,   -220,   -219,   -218,   -217,
  -216,   -215,   -214,   -213,   -212,   -211,   -210,   -209,   -208,   -207,
  -206,   -205,   -204,   -203,   -202,   -201,   -200,   -199,   -198,   -197,
  -196,   -195,   -194,   -193,   -192,   -191,   -190,   -189,   -188,   -187,
  -186,   -185,   -184,   -183,   -182,   -181,   -180,   -179,   -178,   -177,
  -176,   -175,   -174,   -173,   -172,   -171,   -170,   -169,   -168,   -167,
  -166,   -165,   -164,   -163,   -162,   -161,   -160,   -159,   -158,   -157,
  -156,   -155,   -154,   -153,   -152,   -151,   -150,   -149,   -148,   -147,
  -146,   -145,   -144,   -143,   -142,   -141,   -140,   -139,   -138,   -137,
  -136,   -135,   -134,   -133,   -132,   -131,   -130,   -129,   -128,   -127,
  -126,   -125,   -124,   -123,   -122,   -121,   -120,   -119,   -118,   -117,
  -116,   -115,   -114,   -113,   -112,   -111,   -110,   -109,   -108,   -107,
  -106,   -105,   -104,   -103,   -102,   -101,   -100,    -99,    -98,    -97,
   -96,    -95,    -94,    -93,    -92,    -91,    -90,    -89,    -88,    -87,
   -86,    -85,    -84,    -83,    -82,    -81,    -80,    -79,    -78,    -77,
   -76,    -75,    -74,    -73,    -72,    -71,    -70,    -69,    -68,    -67,
   -66,    -65,    -64,    -63,    -62,    -61,    -60,    -59,    -58,    -57,
   -56,    -55,    -54,    -53,    -52,    -51,    -50,    -49,    -48,    -47,
   -46,    -45,    -44,    -43,    -42,    -41,    -40,    -39,    -38,    -37,
   -36,    -35,    -34,    -33,    -32,    -31,    -30,    -29,    -28,    -27,
   -26,    -25,    -24,    -23,    -22,    -21,    -20,    -19,    -18,    -17,
   -16,    -15,    -14,    -13,    -12,    -11,    -10,     -9,     -8,     -7,
    -6,     -5,     -4,     -3,     -2,     -1, };

short translate_lp_frame_index[5760] = {
   1,    0,    9,    5,    4,   13,    8,   17,   16,   12,
  21,   20,   25,   24,   33,   29,   28,   37,   32,   41,
  40,   36,   45,   44,   49,   48,   57,   53,   52,   61,
  56,   65,   64,   60,   69,   68,   73,   72,   81,   77,
  76,   85,   80,   89,   88,   84,   93,   92,   97,   96,
 105,  101,  100,  109,  104,  113,  112,  108,  117,  116,
 121,  120,  129,  125,  124,  133,  128,  137,  136,  132,
 141,  140,  145,  144,  153,  149,  148,  157,  152,  161,
 160,  156,  165,  164,  169,  168,  177,  173,  172,  181,
 176,  185,  184,  180,  189,  188,  193,  192,  201,  197,
 196,  205,  200,  209,  208,  204,  213,  212,  217,  216,
 225,  221,  220,  229,  224,  233,  232,  228,  237,  236,
 241,  240,  249,  245,  244,  253,  248,  257,  256,  252,
 261,  260,  265,  264,  273,  269,  268,  277,  272,  281,
 280,  276,  285,  284,  289,  288,  297,  293,  292,  301,
 296,  305,  304,  300,  309,  308,  313,  312,  321,  317,
 316,  325,  320,  329,  328,  324,  333,  332,  337,  336,
 345,  341,  340,  349,  344,  353,  352,  348,  357,  356,
 361,  360,  369,  365,  364,  373,  368,  377,  376,  372,
 381,  380,  385,  384,  393,  389,  388,  397,  392,  401,
 400,  396,  405,  404,  409,  408,  417,  413,  412,  421,
 416,  425,  424,  420,  429,  428,  433,  432,  441,  437,
 436,  445,  440,  449,  448,  444,  453,  452,  457,  456,
 465,  461,  460,  469,  464,  473,  472,  468,  477,  476,
 481,  480,  489,  485,  484,  493,  488,  497,  496,  492,
 501,  500,  505,  504,  513,  509,  508,  517,  512,  521,
 520,  516,  525,  524,  529,  528,  537,  533,  532,  541,
 536,  545,  544,  540,  549,  548,  553,  552,  561,  557,
 556,  565,  560,  569,  568,  564,  573,  572,  577,  576,
 585,  581,  580,  589,  584,  593,  592,  588,  597,  596,
 601,  600,  609,  605,  604,  613,  608,  617,  616,  612,
 621,  620,  625,  624,  633,  629,  628,  637,  632,  641,
 640,  636,  645,  644,  649,  648,  657,  653,  652,  661,
 656,  665,  664,  660,  669,  668,  673,  672,  681,  677,
 676,  685,  680,  689,  688,  684,  693,  692,  697,  696,
 705,  701,  700,  709,  704,  713,  712,  708,  717,  716,
 721,  720,  729,  725,  724,  733,  728,  737,  736,  732,
 741,  740,  745,  744,  753,  749,  748,  757,  752,  761,
 760,  756,  765,  764,  769,  768,  777,  773,  772,  781,
 776,  785,  784,  780,  789,  788,  793,  792,  801,  797,
 796,  805,  800,  809,  808,  804,  813,  812,  817,  816,
 825,  821,  820,  829,  824,  833,  832,  828,  837,  836,
 841,  840,  849,  845,  844,  853,  848,  857,  856,  852,
 861,  860,  865,  864,  873,  869,  868,  877,  872,  881,
 880,  876,  885,  884,  889,  888,  897,  893,  892,  901,
 896,  905,  904,  900,  909,  908,  913,  912,  921,  917,
 916,  925,  920,  929,  928,  924,  933,  932,  937,  936,
 945,  941,  940,  949,  944,  953,  952,  948,  957,  956,
 961,  960,  969,  965,  964,  973,  968,  977,  976,  972,
 981,  980,  985,  984,  993,  989,  988,  997,  992, 1001,
1000,  996, 1005, 1004, 1009, 1008, 1017, 1013, 1012, 1021,
1016, 1025, 1024, 1020, 1029, 1028, 1033, 1032, 1041, 1037,
1036, 1045, 1040, 1049, 1048, 1044, 1053, 1052, 1057, 1056,
1065, 1061, 1060, 1069, 1064, 1073, 1072, 1068, 1077, 1076,
1081, 1080, 1089, 1085, 1084, 1093, 1088, 1097, 1096, 1092,
1101, 1100, 1105, 1104, 1113, 1109, 1108, 1117, 1112, 1121,
1120, 1116, 1125, 1124, 1129, 1128, 1137, 1133, 1132, 1141,
1136, 1145, 1144, 1140, 1149, 1148, 1153, 1152, 1161, 1157,
1156, 1165, 1160, 1169, 1168, 1164, 1173, 1172, 1177, 1176,
1185, 1181, 1180, 1189, 1184, 1193, 1192, 1188, 1197, 1196,
1201, 1200, 1209, 1205, 1204, 1213, 1208, 1217, 1216, 1212,
1221, 1220, 1225, 1224, 1233, 1229, 1228, 1237, 1232, 1241,
1240, 1236, 1245, 1244, 1249, 1248, 1257, 1253, 1252, 1261,
1256, 1265, 1264, 1260, 1269, 1268, 1273, 1272, 1281, 1277,
1276, 1285, 1280, 1289, 1288, 1284, 1293, 1292, 1297, 1296,
1305, 1301, 1300, 1309, 1304, 1313, 1312, 1308, 1317, 1316,
1321, 1320, 1329, 1325, 1324, 1333, 1328, 1337, 1336, 1332,
1341, 1340, 1345, 1344, 1353, 1349, 1348, 1357, 1352, 1361,
1360, 1356, 1365, 1364, 1369, 1368, 1377, 1373, 1372, 1381,
1376, 1385, 1384, 1380, 1389, 1388, 1393, 1392, 1401, 1397,
1396, 1405, 1400, 1409, 1408, 1404, 1413, 1412, 1417, 1416,
1425, 1421, 1420, 1429, 1424, 1433, 1432, 1428, 1437, 1436,
1441, 1440, 1449, 1445, 1444, 1453, 1448, 1457, 1456, 1452,
1461, 1460, 1465, 1464, 1473, 1469, 1468, 1477, 1472, 1481,
1480, 1476, 1485, 1484, 1489, 1488, 1497, 1493, 1492, 1501,
1496, 1505, 1504, 1500, 1509, 1508, 1513, 1512, 1521, 1517,
1516, 1525, 1520, 1529, 1528, 1524, 1533, 1532, 1537, 1536,
1545, 1541, 1540, 1549, 1544, 1553, 1552, 1548, 1557, 1556,
1561, 1560, 1569, 1565, 1564, 1573, 1568, 1577, 1576, 1572,
1581, 1580, 1585, 1584, 1593, 1589, 1588, 1597, 1592, 1601,
1600, 1596, 1605, 1604, 1609, 1608, 1617, 1613, 1612, 1621,
1616, 1625, 1624, 1620, 1629, 1628, 1633, 1632, 1641, 1637,
1636, 1645, 1640, 1649, 1648, 1644, 1653, 1652, 1657, 1656,
1665, 1661, 1660, 1669, 1664, 1673, 1672, 1668, 1677, 1676,
1681, 1680, 1689, 1685, 1684, 1693, 1688, 1697, 1696, 1692,
1701, 1700, 1705, 1704, 1713, 1709, 1708, 1717, 1712, 1721,
1720, 1716, 1725, 1724, 1729, 1728, 1737, 1733, 1732, 1741,
1736, 1745, 1744, 1740, 1749, 1748, 1753, 1752, 1761, 1757,
1756, 1765, 1760, 1769, 1768, 1764, 1773, 1772, 1777, 1776,
1785, 1781, 1780, 1789, 1784, 1793, 1792, 1788, 1797, 1796,
1801, 1800, 1809, 1805, 1804, 1813, 1808, 1817, 1816, 1812,
1821, 1820, 1825, 1824, 1833, 1829, 1828, 1837, 1832, 1841,
1840, 1836, 1845, 1844, 1849, 1848, 1857, 1853, 1852, 1861,
1856, 1865, 1864, 1860, 1869, 1868, 1873, 1872, 1881, 1877,
1876, 1885, 1880, 1889, 1888, 1884, 1893, 1892, 1897, 1896,
1905, 1901, 1900, 1909, 1904, 1913, 1912, 1908, 1917, 1916,
1921, 1920, 1929, 1925, 1924, 1933, 1928, 1937, 1936, 1932,
1941, 1940, 1945, 1944, 1953, 1949, 1948, 1957, 1952, 1961,
1960, 1956, 1965, 1964, 1969, 1968, 1977, 1973, 1972, 1981,
1976, 1985, 1984, 1980, 1989, 1988, 1993, 1992, 2001, 1997,
1996, 2005, 2000, 2009, 2008, 2004, 2013, 2012, 2017, 2016,
2025, 2021, 2020, 2029, 2024, 2033, 2032, 2028, 2037, 2036,
2041, 2040, 2049, 2045, 2044, 2053, 2048, 2057, 2056, 2052,
2061, 2060, 2065, 2064, 2073, 2069, 2068, 2077, 2072, 2081,
2080, 2076, 2085, 2084, 2089, 2088, 2097, 2093, 2092, 2101,
2096, 2105, 2104, 2100, 2109, 2108, 2113, 2112, 2121, 2117,
2116, 2125, 2120, 2129, 2128, 2124, 2133, 2132, 2137, 2136,
2145, 2141, 2140, 2149, 2144, 2153, 2152, 2148, 2157, 2156,
2161, 2160, 2169, 2165, 2164, 2173, 2168, 2177, 2176, 2172,
2181, 2180, 2185, 2184, 2193, 2189, 2188, 2197, 2192, 2201,
2200, 2196, 2205, 2204, 2209, 2208, 2217, 2213, 2212, 2221,
2216, 2225, 2224, 2220, 2229, 2228, 2233, 2232, 2241, 2237,
2236, 2245, 2240, 2249, 2248, 2244, 2253, 2252, 2257, 2256,
2265, 2261, 2260, 2269, 2264, 2273, 2272, 2268, 2277, 2276,
2281, 2280, 2289, 2285, 2284, 2293, 2288, 2297, 2296, 2292,
2301, 2300, 2305, 2304, 2313, 2309, 2308, 2317, 2312, 2321,
2320, 2316, 2325, 2324, 2329, 2328, 2337, 2333, 2332, 2341,
2336, 2345, 2344, 2340, 2349, 2348, 2353, 2352, 2361, 2357,
2356, 2365, 2360, 2369, 2368, 2364, 2373, 2372, 2377, 2376,
2385, 2381, 2380, 2389, 2384, 2393, 2392, 2388, 2397, 2396,
2401, 2400, 2409, 2405, 2404, 2413, 2408, 2417, 2416, 2412,
2421, 2420, 2425, 2424, 2433, 2429, 2428, 2437, 2432, 2441,
2440, 2436, 2445, 2444, 2449, 2448, 2457, 2453, 2452, 2461,
2456, 2465, 2464, 2460, 2469, 2468, 2473, 2472, 2481, 2477,
2476, 2485, 2480, 2489, 2488, 2484, 2493, 2492, 2497, 2496,
2505, 2501, 2500, 2509, 2504, 2513, 2512, 2508, 2517, 2516,
2521, 2520, 2529, 2525, 2524, 2533, 2528, 2537, 2536, 2532,
2541, 2540, 2545, 2544, 2553, 2549, 2548, 2557, 2552, 2561,
2560, 2556, 2565, 2564, 2569, 2568, 2577, 2573, 2572, 2581,
2576, 2585, 2584, 2580, 2589, 2588, 2593, 2592, 2601, 2597,
2596, 2605, 2600, 2609, 2608, 2604, 2613, 2612, 2617, 2616,
2625, 2621, 2620, 2629, 2624, 2633, 2632, 2628, 2637, 2636,
2641, 2640, 2649, 2645, 2644, 2653, 2648, 2657, 2656, 2652,
2661, 2660, 2665, 2664, 2673, 2669, 2668, 2677, 2672, 2681,
2680, 2676, 2685, 2684, 2689, 2688, 2697, 2693, 2692, 2701,
2696, 2705, 2704, 2700, 2709, 2708, 2713, 2712, 2721, 2717,
2716, 2725, 2720, 2729, 2728, 2724, 2733, 2732, 2737, 2736,
2745, 2741, 2740, 2749, 2744, 2753, 2752, 2748, 2757, 2756,
2761, 2760, 2769, 2765, 2764, 2773, 2768, 2777, 2776, 2772,
2781, 2780, 2785, 2784, 2793, 2789, 2788, 2797, 2792, 2801,
2800, 2796, 2805, 2804, 2809, 2808, 2817, 2813, 2812, 2821,
2816, 2825, 2824, 2820, 2829, 2828, 2833, 2832, 2841, 2837,
2836, 2845, 2840, 2849, 2848, 2844, 2853, 2852, 2857, 2856,
2865, 2861, 2860, 2869, 2864, 2873, 2872, 2868, 2877, 2876,
2881, 2880, 2889, 2885, 2884, 2893, 2888, 2897, 2896, 2892,
2901, 2900, 2905, 2904, 2913, 2909, 2908, 2917, 2912, 2921,
2920, 2916, 2925, 2924, 2929, 2928, 2937, 2933, 2932, 2941,
2936, 2945, 2944, 2940, 2949, 2948, 2953, 2952, 2961, 2957,
2956, 2965, 2960, 2969, 2968, 2964, 2973, 2972, 2977, 2976,
2985, 2981, 2980, 2989, 2984, 2993, 2992, 2988, 2997, 2996,
3001, 3000, 3009, 3005, 3004, 3013, 3008, 3017, 3016, 3012,
3021, 3020, 3025, 3024, 3033, 3029, 3028, 3037, 3032, 3041,
3040, 3036, 3045, 3044, 3049, 3048, 3057, 3053, 3052, 3061,
3056, 3065, 3064, 3060, 3069, 3068, 3073, 3072, 3081, 3077,
3076, 3085, 3080, 3089, 3088, 3084, 3093, 3092, 3097, 3096,
3105, 3101, 3100, 3109, 3104, 3113, 3112, 3108, 3117, 3116,
3121, 3120, 3129, 3125, 3124, 3133, 3128, 3137, 3136, 3132,
3141, 3140, 3145, 3144, 3153, 3149, 3148, 3157, 3152, 3161,
3160, 3156, 3165, 3164, 3169, 3168, 3177, 3173, 3172, 3181,
3176, 3185, 3184, 3180, 3189, 3188, 3193, 3192, 3201, 3197,
3196, 3205, 3200, 3209, 3208, 3204, 3213, 3212, 3217, 3216,
3225, 3221, 3220, 3229, 3224, 3233, 3232, 3228, 3237, 3236,
3241, 3240, 3249, 3245, 3244, 3253, 3248, 3257, 3256, 3252,
3261, 3260, 3265, 3264, 3273, 3269, 3268, 3277, 3272, 3281,
3280, 3276, 3285, 3284, 3289, 3288, 3297, 3293, 3292, 3301,
3296, 3305, 3304, 3300, 3309, 3308, 3313, 3312, 3321, 3317,
3316, 3325, 3320, 3329, 3328, 3324, 3333, 3332, 3337, 3336,
3345, 3341, 3340, 3349, 3344, 3353, 3352, 3348, 3357, 3356,
3361, 3360, 3369, 3365, 3364, 3373, 3368, 3377, 3376, 3372,
3381, 3380, 3385, 3384, 3393, 3389, 3388, 3397, 3392, 3401,
3400, 3396, 3405, 3404, 3409, 3408, 3417, 3413, 3412, 3421,
3416, 3425, 3424, 3420, 3429, 3428, 3433, 3432, 3441, 3437,
3436, 3445, 3440, 3449, 3448, 3444, 3453, 3452, 3457, 3456,
3465, 3461, 3460, 3469, 3464, 3473, 3472, 3468, 3477, 3476,
3481, 3480, 3489, 3485, 3484, 3493, 3488, 3497, 3496, 3492,
3501, 3500, 3505, 3504, 3513, 3509, 3508, 3517, 3512, 3521,
3520, 3516, 3525, 3524, 3529, 3528, 3537, 3533, 3532, 3541,
3536, 3545, 3544, 3540, 3549, 3548, 3553, 3552, 3561, 3557,
3556, 3565, 3560, 3569, 3568, 3564, 3573, 3572, 3577, 3576,
3585, 3581, 3580, 3589, 3584, 3593, 3592, 3588, 3597, 3596,
3601, 3600, 3609, 3605, 3604, 3613, 3608, 3617, 3616, 3612,
3621, 3620, 3625, 3624, 3633, 3629, 3628, 3637, 3632, 3641,
3640, 3636, 3645, 3644, 3649, 3648, 3657, 3653, 3652, 3661,
3656, 3665, 3664, 3660, 3669, 3668, 3673, 3672, 3681, 3677,
3676, 3685, 3680, 3689, 3688, 3684, 3693, 3692, 3697, 3696,
3705, 3701, 3700, 3709, 3704, 3713, 3712, 3708, 3717, 3716,
3721, 3720, 3729, 3725, 3724, 3733, 3728, 3737, 3736, 3732,
3741, 3740, 3745, 3744, 3753, 3749, 3748, 3757, 3752, 3761,
3760, 3756, 3765, 3764, 3769, 3768, 3777, 3773, 3772, 3781,
3776, 3785, 3784, 3780, 3789, 3788, 3793, 3792, 3801, 3797,
3796, 3805, 3800, 3809, 3808, 3804, 3813, 3812, 3817, 3816,
3825, 3821, 3820, 3829, 3824, 3833, 3832, 3828, 3837, 3836,
3841, 3840, 3849, 3845, 3844, 3853, 3848, 3857, 3856, 3852,
3861, 3860, 3865, 3864, 3873, 3869, 3868, 3877, 3872, 3881,
3880, 3876, 3885, 3884, 3889, 3888, 3897, 3893, 3892, 3901,
3896, 3905, 3904, 3900, 3909, 3908, 3913, 3912, 3921, 3917,
3916, 3925, 3920, 3929, 3928, 3924, 3933, 3932, 3937, 3936,
3945, 3941, 3940, 3949, 3944, 3953, 3952, 3948, 3957, 3956,
3961, 3960, 3969, 3965, 3964, 3973, 3968, 3977, 3976, 3972,
3981, 3980, 3985, 3984, 3993, 3989, 3988, 3997, 3992, 4001,
4000, 3996, 4005, 4004, 4009, 4008, 4017, 4013, 4012, 4021,
4016, 4025, 4024, 4020, 4029, 4028, 4033, 4032, 4041, 4037,
4036, 4045, 4040, 4049, 4048, 4044, 4053, 4052, 4057, 4056,
4065, 4061, 4060, 4069, 4064, 4073, 4072, 4068, 4077, 4076,
4081, 4080, 4089, 4085, 4084, 4093, 4088, 4097, 4096, 4092,
4101, 4100, 4105, 4104, 4113, 4109, 4108, 4117, 4112, 4121,
4120, 4116, 4125, 4124, 4129, 4128, 4137, 4133, 4132, 4141,
4136, 4145, 4144, 4140, 4149, 4148, 4153, 4152, 4161, 4157,
4156, 4165, 4160, 4169, 4168, 4164, 4173, 4172, 4177, 4176,
4185, 4181, 4180, 4189, 4184, 4193, 4192, 4188, 4197, 4196,
4201, 4200, 4209, 4205, 4204, 4213, 4208, 4217, 4216, 4212,
4221, 4220, 4225, 4224, 4233, 4229, 4228, 4237, 4232, 4241,
4240, 4236, 4245, 4244, 4249, 4248, 4257, 4253, 4252, 4261,
4256, 4265, 4264, 4260, 4269, 4268, 4273, 4272, 4281, 4277,
4276, 4285, 4280, 4289, 4288, 4284, 4293, 4292, 4297, 4296,
4305, 4301, 4300, 4309, 4304, 4313, 4312, 4308, 4317, 4316,
4321, 4320, 4329, 4325, 4324, 4333, 4328, 4337, 4336, 4332,
4341, 4340, 4345, 4344, 4353, 4349, 4348, 4357, 4352, 4361,
4360, 4356, 4365, 4364, 4369, 4368, 4377, 4373, 4372, 4381,
4376, 4385, 4384, 4380, 4389, 4388, 4393, 4392, 4401, 4397,
4396, 4405, 4400, 4409, 4408, 4404, 4413, 4412, 4417, 4416,
4425, 4421, 4420, 4429, 4424, 4433, 4432, 4428, 4437, 4436,
4441, 4440, 4449, 4445, 4444, 4453, 4448, 4457, 4456, 4452,
4461, 4460, 4465, 4464, 4473, 4469, 4468, 4477, 4472, 4481,
4480, 4476, 4485, 4484, 4489, 4488, 4497, 4493, 4492, 4501,
4496, 4505, 4504, 4500, 4509, 4508, 4513, 4512, 4521, 4517,
4516, 4525, 4520, 4529, 4528, 4524, 4533, 4532, 4537, 4536,
4545, 4541, 4540, 4549, 4544, 4553, 4552, 4548, 4557, 4556,
4561, 4560, 4569, 4565, 4564, 4573, 4568, 4577, 4576, 4572,
4581, 4580, 4585, 4584, 4593, 4589, 4588, 4597, 4592, 4601,
4600, 4596, 4605, 4604, 4609, 4608, 4617, 4613, 4612, 4621,
4616, 4625, 4624, 4620, 4629, 4628, 4633, 4632, 4641, 4637,
4636, 4645, 4640, 4649, 4648, 4644, 4653, 4652, 4657, 4656,
4665, 4661, 4660, 4669, 4664, 4673, 4672, 4668, 4677, 4676,
4681, 4680, 4689, 4685, 4684, 4693, 4688, 4697, 4696, 4692,
4701, 4700, 4705, 4704, 4713, 4709, 4708, 4717, 4712, 4721,
4720, 4716, 4725, 4724, 4729, 4728, 4737, 4733, 4732, 4741,
4736, 4745, 4744, 4740, 4749, 4748, 4753, 4752, 4761, 4757,
4756, 4765, 4760, 4769, 4768, 4764, 4773, 4772, 4777, 4776,
4785, 4781, 4780, 4789, 4784, 4793, 4792, 4788, 4797, 4796,
4801, 4800, 4809, 4805, 4804, 4813, 4808, 4817, 4816, 4812,
4821, 4820, 4825, 4824, 4833, 4829, 4828, 4837, 4832, 4841,
4840, 4836, 4845, 4844, 4849, 4848, 4857, 4853, 4852, 4861,
4856, 4865, 4864, 4860, 4869, 4868, 4873, 4872, 4881, 4877,
4876, 4885, 4880, 4889, 4888, 4884, 4893, 4892, 4897, 4896,
4905, 4901, 4900, 4909, 4904, 4913, 4912, 4908, 4917, 4916,
4921, 4920, 4929, 4925, 4924, 4933, 4928, 4937, 4936, 4932,
4941, 4940, 4945, 4944, 4953, 4949, 4948, 4957, 4952, 4961,
4960, 4956, 4965, 4964, 4969, 4968, 4977, 4973, 4972, 4981,
4976, 4985, 4984, 4980, 4989, 4988, 4993, 4992, 5001, 4997,
4996, 5005, 5000, 5009, 5008, 5004, 5013, 5012, 5017, 5016,
5025, 5021, 5020, 5029, 5024, 5033, 5032, 5028, 5037, 5036,
5041, 5040, 5049, 5045, 5044, 5053, 5048, 5057, 5056, 5052,
5061, 5060, 5065, 5064, 5073, 5069, 5068, 5077, 5072, 5081,
5080, 5076, 5085, 5084, 5089, 5088, 5097, 5093, 5092, 5101,
5096, 5105, 5104, 5100, 5109, 5108, 5113, 5112, 5121, 5117,
5116, 5125, 5120, 5129, 5128, 5124, 5133, 5132, 5137, 5136,
5145, 5141, 5140, 5149, 5144, 5153, 5152, 5148, 5157, 5156,
5161, 5160, 5169, 5165, 5164, 5173, 5168, 5177, 5176, 5172,
5181, 5180, 5185, 5184, 5193, 5189, 5188, 5197, 5192, 5201,
5200, 5196, 5205, 5204, 5209, 5208, 5217, 5213, 5212, 5221,
5216, 5225, 5224, 5220, 5229, 5228, 5233, 5232, 5241, 5237,
5236, 5245, 5240, 5249, 5248, 5244, 5253, 5252, 5257, 5256,
5265, 5261, 5260, 5269, 5264, 5273, 5272, 5268, 5277, 5276,
5281, 5280, 5289, 5285, 5284, 5293, 5288, 5297, 5296, 5292,
5301, 5300, 5305, 5304, 5313, 5309, 5308, 5317, 5312, 5321,
5320, 5316, 5325, 5324, 5329, 5328, 5337, 5333, 5332, 5341,
5336, 5345, 5344, 5340, 5349, 5348, 5353, 5352, 5361, 5357,
5356, 5365, 5360, 5369, 5368, 5364, 5373, 5372, 5377, 5376,
5385, 5381, 5380, 5389, 5384, 5393, 5392, 5388, 5397, 5396,
5401, 5400, 5409, 5405, 5404, 5413, 5408, 5417, 5416, 5412,
5421, 5420, 5425, 5424, 5433, 5429, 5428, 5437, 5432, 5441,
5440, 5436, 5445, 5444, 5449, 5448, 5457, 5453, 5452, 5461,
5456, 5465, 5464, 5460, 5469, 5468, 5473, 5472, 5481, 5477,
5476, 5485, 5480, 5489, 5488, 5484, 5493, 5492, 5497, 5496,
5505, 5501, 5500, 5509, 5504, 5513, 5512, 5508, 5517, 5516,
5521, 5520, 5529, 5525, 5524, 5533, 5528, 5537, 5536, 5532,
5541, 5540, 5545, 5544, 5553, 5549, 5548, 5557, 5552, 5561,
5560, 5556, 5565, 5564, 5569, 5568, 5577, 5573, 5572, 5581,
5576, 5585, 5584, 5580, 5589, 5588, 5593, 5592, 5601, 5597,
5596, 5605, 5600, 5609, 5608, 5604, 5613, 5612, 5617, 5616,
5625, 5621, 5620, 5629, 5624, 5633, 5632, 5628, 5637, 5636,
5641, 5640, 5649, 5645, 5644, 5653, 5648, 5657, 5656, 5652,
5661, 5660, 5665, 5664, 5673, 5669, 5668, 5677, 5672, 5681,
5680, 5676, 5685, 5684, 5689, 5688, 5697, 5693, 5692, 5701,
5696, 5705, 5704, 5700, 5709, 5708, 5713, 5712, 5721, 5717,
5716, 5725, 5720, 5729, 5728, 5724, 5733, 5732, 5737, 5736,
5745, 5741, 5740, 5749, 5744, 5753, 5752, 5748, 5757, 5756,
   3,    2,   11,    7,    6,   15,   10,   19,   18,   14,
  23,   22,   27,   26,   35,   31,   30,   39,   34,   43,
  42,   38,   47,   46,   51,   50,   59,   55,   54,   63,
  58,   67,   66,   62,   71,   70,   75,   74,   83,   79,
  78,   87,   82,   91,   90,   86,   95,   94,   99,   98,
 107,  103,  102,  111,  106,  115,  114,  110,  119,  118,
 123,  122,  131,  127,  126,  135,  130,  139,  138,  134,
 143,  142,  147,  146,  155,  151,  150,  159,  154,  163,
 162,  158,  167,  166,  171,  170,  179,  175,  174,  183,
 178,  187,  186,  182,  191,  190,  195,  194,  203,  199,
 198,  207,  202,  211,  210,  206,  215,  214,  219,  218,
 227,  223,  222,  231,  226,  235,  234,  230,  239,  238,
 243,  242,  251,  247,  246,  255,  250,  259,  258,  254,
 263,  262,  267,  266,  275,  271,  270,  279,  274,  283,
 282,  278,  287,  286,  291,  290,  299,  295,  294,  303,
 298,  307,  306,  302,  311,  310,  315,  314,  323,  319,
 318,  327,  322,  331,  330,  326,  335,  334,  339,  338,
 347,  343,  342,  351,  346,  355,  354,  350,  359,  358,
 363,  362,  371,  367,  366,  375,  370,  379,  378,  374,
 383,  382,  387,  386,  395,  391,  390,  399,  394,  403,
 402,  398,  407,  406,  411,  410,  419,  415,  414,  423,
 418,  427,  426,  422,  431,  430,  435,  434,  443,  439,
 438,  447,  442,  451,  450,  446,  455,  454,  459,  458,
 467,  463,  462,  471,  466,  475,  474,  470,  479,  478,
 483,  482,  491,  487,  486,  495,  490,  499,  498,  494,
 503,  502,  507,  506,  515,  511,  510,  519,  514,  523,
 522,  518,  527,  526,  531,  530,  539,  535,  534,  543,
 538,  547,  546,  542,  551,  550,  555,  554,  563,  559,
 558,  567,  562,  571,  570,  566,  575,  574,  579,  578,
 587,  583,  582,  591,  586,  595,  594,  590,  599,  598,
 603,  602,  611,  607,  606,  615,  610,  619,  618,  614,
 623,  622,  627,  626,  635,  631,  630,  639,  634,  643,
 642,  638,  647,  646,  651,  650,  659,  655,  654,  663,
 658,  667,  666,  662,  671,  670,  675,  674,  683,  679,
 678,  687,  682,  691,  690,  686,  695,  694,  699,  698,
 707,  703,  702,  711,  706,  715,  714,  710,  719,  718,
 723,  722,  731,  727,  726,  735,  730,  739,  738,  734,
 743,  742,  747,  746,  755,  751,  750,  759,  754,  763,
 762,  758,  767,  766,  771,  770,  779,  775,  774,  783,
 778,  787,  786,  782,  791,  790,  795,  794,  803,  799,
 798,  807,  802,  811,  810,  806,  815,  814,  819,  818,
 827,  823,  822,  831,  826,  835,  834,  830,  839,  838,
 843,  842,  851,  847,  846,  855,  850,  859,  858,  854,
 863,  862,  867,  866,  875,  871,  870,  879,  874,  883,
 882,  878,  887,  886,  891,  890,  899,  895,  894,  903,
 898,  907,  906,  902,  911,  910,  915,  914,  923,  919,
 918,  927,  922,  931,  930,  926,  935,  934,  939,  938,
 947,  943,  942,  951,  946,  955,  954,  950,  959,  958,
 963,  962,  971,  967,  966,  975,  970,  979,  978,  974,
 983,  982,  987,  986,  995,  991,  990,  999,  994, 1003,
1002,  998, 1007, 1006, 1011, 1010, 1019, 1015, 1014, 1023,
1018, 1027, 1026, 1022, 1031, 1030, 1035, 1034, 1043, 1039,
1038, 1047, 1042, 1051, 1050, 1046, 1055, 1054, 1059, 1058,
1067, 1063, 1062, 1071, 1066, 1075, 1074, 1070, 1079, 1078,
1083, 1082, 1091, 1087, 1086, 1095, 1090, 1099, 1098, 1094,
1103, 1102, 1107, 1106, 1115, 1111, 1110, 1119, 1114, 1123,
1122, 1118, 1127, 1126, 1131, 1130, 1139, 1135, 1134, 1143,
1138, 1147, 1146, 1142, 1151, 1150, 1155, 1154, 1163, 1159,
1158, 1167, 1162, 1171, 1170, 1166, 1175, 1174, 1179, 1178,
1187, 1183, 1182, 1191, 1186, 1195, 1194, 1190, 1199, 1198,
1203, 1202, 1211, 1207, 1206, 1215, 1210, 1219, 1218, 1214,
1223, 1222, 1227, 1226, 1235, 1231, 1230, 1239, 1234, 1243,
1242, 1238, 1247, 1246, 1251, 1250, 1259, 1255, 1254, 1263,
1258, 1267, 1266, 1262, 1271, 1270, 1275, 1274, 1283, 1279,
1278, 1287, 1282, 1291, 1290, 1286, 1295, 1294, 1299, 1298,
1307, 1303, 1302, 1311, 1306, 1315, 1314, 1310, 1319, 1318,
1323, 1322, 1331, 1327, 1326, 1335, 1330, 1339, 1338, 1334,
1343, 1342, 1347, 1346, 1355, 1351, 1350, 1359, 1354, 1363,
1362, 1358, 1367, 1366, 1371, 1370, 1379, 1375, 1374, 1383,
1378, 1387, 1386, 1382, 1391, 1390, 1395, 1394, 1403, 1399,
1398, 1407, 1402, 1411, 1410, 1406, 1415, 1414, 1419, 1418,
1427, 1423, 1422, 1431, 1426, 1435, 1434, 1430, 1439, 1438,
1443, 1442, 1451, 1447, 1446, 1455, 1450, 1459, 1458, 1454,
1463, 1462, 1467, 1466, 1475, 1471, 1470, 1479, 1474, 1483,
1482, 1478, 1487, 1486, 1491, 1490, 1499, 1495, 1494, 1503,
1498, 1507, 1506, 1502, 1511, 1510, 1515, 1514, 1523, 1519,
1518, 1527, 1522, 1531, 1530, 1526, 1535, 1534, 1539, 1538,
1547, 1543, 1542, 1551, 1546, 1555, 1554, 1550, 1559, 1558,
1563, 1562, 1571, 1567, 1566, 1575, 1570, 1579, 1578, 1574,
1583, 1582, 1587, 1586, 1595, 1591, 1590, 1599, 1594, 1603,
1602, 1598, 1607, 1606, 1611, 1610, 1619, 1615, 1614, 1623,
1618, 1627, 1626, 1622, 1631, 1630, 1635, 1634, 1643, 1639,
1638, 1647, 1642, 1651, 1650, 1646, 1655, 1654, 1659, 1658,
1667, 1663, 1662, 1671, 1666, 1675, 1674, 1670, 1679, 1678,
1683, 1682, 1691, 1687, 1686, 1695, 1690, 1699, 1698, 1694,
1703, 1702, 1707, 1706, 1715, 1711, 1710, 1719, 1714, 1723,
1722, 1718, 1727, 1726, 1731, 1730, 1739, 1735, 1734, 1743,
1738, 1747, 1746, 1742, 1751, 1750, 1755, 1754, 1763, 1759,
1758, 1767, 1762, 1771, 1770, 1766, 1775, 1774, 1779, 1778,
1787, 1783, 1782, 1791, 1786, 1795, 1794, 1790, 1799, 1798,
1803, 1802, 1811, 1807, 1806, 1815, 1810, 1819, 1818, 1814,
1823, 1822, 1827, 1826, 1835, 1831, 1830, 1839, 1834, 1843,
1842, 1838, 1847, 1846, 1851, 1850, 1859, 1855, 1854, 1863,
1858, 1867, 1866, 1862, 1871, 1870, 1875, 1874, 1883, 1879,
1878, 1887, 1882, 1891, 1890, 1886, 1895, 1894, 1899, 1898,
1907, 1903, 1902, 1911, 1906, 1915, 1914, 1910, 1919, 1918,
1923, 1922, 1931, 1927, 1926, 1935, 1930, 1939, 1938, 1934,
1943, 1942, 1947, 1946, 1955, 1951, 1950, 1959, 1954, 1963,
1962, 1958, 1967, 1966, 1971, 1970, 1979, 1975, 1974, 1983,
1978, 1987, 1986, 1982, 1991, 1990, 1995, 1994, 2003, 1999,
1998, 2007, 2002, 2011, 2010, 2006, 2015, 2014, 2019, 2018,
2027, 2023, 2022, 2031, 2026, 2035, 2034, 2030, 2039, 2038,
2043, 2042, 2051, 2047, 2046, 2055, 2050, 2059, 2058, 2054,
2063, 2062, 2067, 2066, 2075, 2071, 2070, 2079, 2074, 2083,
2082, 2078, 2087, 2086, 2091, 2090, 2099, 2095, 2094, 2103,
2098, 2107, 2106, 2102, 2111, 2110, 2115, 2114, 2123, 2119,
2118, 2127, 2122, 2131, 2130, 2126, 2135, 2134, 2139, 2138,
2147, 2143, 2142, 2151, 2146, 2155, 2154, 2150, 2159, 2158,
2163, 2162, 2171, 2167, 2166, 2175, 2170, 2179, 2178, 2174,
2183, 2182, 2187, 2186, 2195, 2191, 2190, 2199, 2194, 2203,
2202, 2198, 2207, 2206, 2211, 2210, 2219, 2215, 2214, 2223,
2218, 2227, 2226, 2222, 2231, 2230, 2235, 2234, 2243, 2239,
2238, 2247, 2242, 2251, 2250, 2246, 2255, 2254, 2259, 2258,
2267, 2263, 2262, 2271, 2266, 2275, 2274, 2270, 2279, 2278,
2283, 2282, 2291, 2287, 2286, 2295, 2290, 2299, 2298, 2294,
2303, 2302, 2307, 2306, 2315, 2311, 2310, 2319, 2314, 2323,
2322, 2318, 2327, 2326, 2331, 2330, 2339, 2335, 2334, 2343,
2338, 2347, 2346, 2342, 2351, 2350, 2355, 2354, 2363, 2359,
2358, 2367, 2362, 2371, 2370, 2366, 2375, 2374, 2379, 2378,
2387, 2383, 2382, 2391, 2386, 2395, 2394, 2390, 2399, 2398,
2403, 2402, 2411, 2407, 2406, 2415, 2410, 2419, 2418, 2414,
2423, 2422, 2427, 2426, 2435, 2431, 2430, 2439, 2434, 2443,
2442, 2438, 2447, 2446, 2451, 2450, 2459, 2455, 2454, 2463,
2458, 2467, 2466, 2462, 2471, 2470, 2475, 2474, 2483, 2479,
2478, 2487, 2482, 2491, 2490, 2486, 2495, 2494, 2499, 2498,
2507, 2503, 2502, 2511, 2506, 2515, 2514, 2510, 2519, 2518,
2523, 2522, 2531, 2527, 2526, 2535, 2530, 2539, 2538, 2534,
2543, 2542, 2547, 2546, 2555, 2551, 2550, 2559, 2554, 2563,
2562, 2558, 2567, 2566, 2571, 2570, 2579, 2575, 2574, 2583,
2578, 2587, 2586, 2582, 2591, 2590, 2595, 2594, 2603, 2599,
2598, 2607, 2602, 2611, 2610, 2606, 2615, 2614, 2619, 2618,
2627, 2623, 2622, 2631, 2626, 2635, 2634, 2630, 2639, 2638,
2643, 2642, 2651, 2647, 2646, 2655, 2650, 2659, 2658, 2654,
2663, 2662, 2667, 2666, 2675, 2671, 2670, 2679, 2674, 2683,
2682, 2678, 2687, 2686, 2691, 2690, 2699, 2695, 2694, 2703,
2698, 2707, 2706, 2702, 2711, 2710, 2715, 2714, 2723, 2719,
2718, 2727, 2722, 2731, 2730, 2726, 2735, 2734, 2739, 2738,
2747, 2743, 2742, 2751, 2746, 2755, 2754, 2750, 2759, 2758,
2763, 2762, 2771, 2767, 2766, 2775, 2770, 2779, 2778, 2774,
2783, 2782, 2787, 2786, 2795, 2791, 2790, 2799, 2794, 2803,
2802, 2798, 2807, 2806, 2811, 2810, 2819, 2815, 2814, 2823,
2818, 2827, 2826, 2822, 2831, 2830, 2835, 2834, 2843, 2839,
2838, 2847, 2842, 2851, 2850, 2846, 2855, 2854, 2859, 2858,
2867, 2863, 2862, 2871, 2866, 2875, 2874, 2870, 2879, 2878,
2883, 2882, 2891, 2887, 2886, 2895, 2890, 2899, 2898, 2894,
2903, 2902, 2907, 2906, 2915, 2911, 2910, 2919, 2914, 2923,
2922, 2918, 2927, 2926, 2931, 2930, 2939, 2935, 2934, 2943,
2938, 2947, 2946, 2942, 2951, 2950, 2955, 2954, 2963, 2959,
2958, 2967, 2962, 2971, 2970, 2966, 2975, 2974, 2979, 2978,
2987, 2983, 2982, 2991, 2986, 2995, 2994, 2990, 2999, 2998,
3003, 3002, 3011, 3007, 3006, 3015, 3010, 3019, 3018, 3014,
3023, 3022, 3027, 3026, 3035, 3031, 3030, 3039, 3034, 3043,
3042, 3038, 3047, 3046, 3051, 3050, 3059, 3055, 3054, 3063,
3058, 3067, 3066, 3062, 3071, 3070, 3075, 3074, 3083, 3079,
3078, 3087, 3082, 3091, 3090, 3086, 3095, 3094, 3099, 3098,
3107, 3103, 3102, 3111, 3106, 3115, 3114, 3110, 3119, 3118,
3123, 3122, 3131, 3127, 3126, 3135, 3130, 3139, 3138, 3134,
3143, 3142, 3147, 3146, 3155, 3151, 3150, 3159, 3154, 3163,
3162, 3158, 3167, 3166, 3171, 3170, 3179, 3175, 3174, 3183,
3178, 3187, 3186, 3182, 3191, 3190, 3195, 3194, 3203, 3199,
3198, 3207, 3202, 3211, 3210, 3206, 3215, 3214, 3219, 3218,
3227, 3223, 3222, 3231, 3226, 3235, 3234, 3230, 3239, 3238,
3243, 3242, 3251, 3247, 3246, 3255, 3250, 3259, 3258, 3254,
3263, 3262, 3267, 3266, 3275, 3271, 3270, 3279, 3274, 3283,
3282, 3278, 3287, 3286, 3291, 3290, 3299, 3295, 3294, 3303,
3298, 3307, 3306, 3302, 3311, 3310, 3315, 3314, 3323, 3319,
3318, 3327, 3322, 3331, 3330, 3326, 3335, 3334, 3339, 3338,
3347, 3343, 3342, 3351, 3346, 3355, 3354, 3350, 3359, 3358,
3363, 3362, 3371, 3367, 3366, 3375, 3370, 3379, 3378, 3374,
3383, 3382, 3387, 3386, 3395, 3391, 3390, 3399, 3394, 3403,
3402, 3398, 3407, 3406, 3411, 3410, 3419, 3415, 3414, 3423,
3418, 3427, 3426, 3422, 3431, 3430, 3435, 3434, 3443, 3439,
3438, 3447, 3442, 3451, 3450, 3446, 3455, 3454, 3459, 3458,
3467, 3463, 3462, 3471, 3466, 3475, 3474, 3470, 3479, 3478,
3483, 3482, 3491, 3487, 3486, 3495, 3490, 3499, 3498, 3494,
3503, 3502, 3507, 3506, 3515, 3511, 3510, 3519, 3514, 3523,
3522, 3518, 3527, 3526, 3531, 3530, 3539, 3535, 3534, 3543,
3538, 3547, 3546, 3542, 3551, 3550, 3555, 3554, 3563, 3559,
3558, 3567, 3562, 3571, 3570, 3566, 3575, 3574, 3579, 3578,
3587, 3583, 3582, 3591, 3586, 3595, 3594, 3590, 3599, 3598,
3603, 3602, 3611, 3607, 3606, 3615, 3610, 3619, 3618, 3614,
3623, 3622, 3627, 3626, 3635, 3631, 3630, 3639, 3634, 3643,
3642, 3638, 3647, 3646, 3651, 3650, 3659, 3655, 3654, 3663,
3658, 3667, 3666, 3662, 3671, 3670, 3675, 3674, 3683, 3679,
3678, 3687, 3682, 3691, 3690, 3686, 3695, 3694, 3699, 3698,
3707, 3703, 3702, 3711, 3706, 3715, 3714, 3710, 3719, 3718,
3723, 3722, 3731, 3727, 3726, 3735, 3730, 3739, 3738, 3734,
3743, 3742, 3747, 3746, 3755, 3751, 3750, 3759, 3754, 3763,
3762, 3758, 3767, 3766, 3771, 3770, 3779, 3775, 3774, 3783,
3778, 3787, 3786, 3782, 3791, 3790, 3795, 3794, 3803, 3799,
3798, 3807, 3802, 3811, 3810, 3806, 3815, 3814, 3819, 3818,
3827, 3823, 3822, 3831, 3826, 3835, 3834, 3830, 3839, 3838,
3843, 3842, 3851, 3847, 3846, 3855, 3850, 3859, 3858, 3854,
3863, 3862, 3867, 3866, 3875, 3871, 3870, 3879, 3874, 3883,
3882, 3878, 3887, 3886, 3891, 3890, 3899, 3895, 3894, 3903,
3898, 3907, 3906, 3902, 3911, 3910, 3915, 3914, 3923, 3919,
3918, 3927, 3922, 3931, 3930, 3926, 3935, 3934, 3939, 3938,
3947, 3943, 3942, 3951, 3946, 3955, 3954, 3950, 3959, 3958,
3963, 3962, 3971, 3967, 3966, 3975, 3970, 3979, 3978, 3974,
3983, 3982, 3987, 3986, 3995, 3991, 3990, 3999, 3994, 4003,
4002, 3998, 4007, 4006, 4011, 4010, 4019, 4015, 4014, 4023,
4018, 4027, 4026, 4022, 4031, 4030, 4035, 4034, 4043, 4039,
4038, 4047, 4042, 4051, 4050, 4046, 4055, 4054, 4059, 4058,
4067, 4063, 4062, 4071, 4066, 4075, 4074, 4070, 4079, 4078,
4083, 4082, 4091, 4087, 4086, 4095, 4090, 4099, 4098, 4094,
4103, 4102, 4107, 4106, 4115, 4111, 4110, 4119, 4114, 4123,
4122, 4118, 4127, 4126, 4131, 4130, 4139, 4135, 4134, 4143,
4138, 4147, 4146, 4142, 4151, 4150, 4155, 4154, 4163, 4159,
4158, 4167, 4162, 4171, 4170, 4166, 4175, 4174, 4179, 4178,
4187, 4183, 4182, 4191, 4186, 4195, 4194, 4190, 4199, 4198,
4203, 4202, 4211, 4207, 4206, 4215, 4210, 4219, 4218, 4214,
4223, 4222, 4227, 4226, 4235, 4231, 4230, 4239, 4234, 4243,
4242, 4238, 4247, 4246, 4251, 4250, 4259, 4255, 4254, 4263,
4258, 4267, 4266, 4262, 4271, 4270, 4275, 4274, 4283, 4279,
4278, 4287, 4282, 4291, 4290, 4286, 4295, 4294, 4299, 4298,
4307, 4303, 4302, 4311, 4306, 4315, 4314, 4310, 4319, 4318,
4323, 4322, 4331, 4327, 4326, 4335, 4330, 4339, 4338, 4334,
4343, 4342, 4347, 4346, 4355, 4351, 4350, 4359, 4354, 4363,
4362, 4358, 4367, 4366, 4371, 4370, 4379, 4375, 4374, 4383,
4378, 4387, 4386, 4382, 4391, 4390, 4395, 4394, 4403, 4399,
4398, 4407, 4402, 4411, 4410, 4406, 4415, 4414, 4419, 4418,
4427, 4423, 4422, 4431, 4426, 4435, 4434, 4430, 4439, 4438,
4443, 4442, 4451, 4447, 4446, 4455, 4450, 4459, 4458, 4454,
4463, 4462, 4467, 4466, 4475, 4471, 4470, 4479, 4474, 4483,
4482, 4478, 4487, 4486, 4491, 4490, 4499, 4495, 4494, 4503,
4498, 4507, 4506, 4502, 4511, 4510, 4515, 4514, 4523, 4519,
4518, 4527, 4522, 4531, 4530, 4526, 4535, 4534, 4539, 4538,
4547, 4543, 4542, 4551, 4546, 4555, 4554, 4550, 4559, 4558,
4563, 4562, 4571, 4567, 4566, 4575, 4570, 4579, 4578, 4574,
4583, 4582, 4587, 4586, 4595, 4591, 4590, 4599, 4594, 4603,
4602, 4598, 4607, 4606, 4611, 4610, 4619, 4615, 4614, 4623,
4618, 4627, 4626, 4622, 4631, 4630, 4635, 4634, 4643, 4639,
4638, 4647, 4642, 4651, 4650, 4646, 4655, 4654, 4659, 4658,
4667, 4663, 4662, 4671, 4666, 4675, 4674, 4670, 4679, 4678,
4683, 4682, 4691, 4687, 4686, 4695, 4690, 4699, 4698, 4694,
4703, 4702, 4707, 4706, 4715, 4711, 4710, 4719, 4714, 4723,
4722, 4718, 4727, 4726, 4731, 4730, 4739, 4735, 4734, 4743,
4738, 4747, 4746, 4742, 4751, 4750, 4755, 4754, 4763, 4759,
4758, 4767, 4762, 4771, 4770, 4766, 4775, 4774, 4779, 4778,
4787, 4783, 4782, 4791, 4786, 4795, 4794, 4790, 4799, 4798,
4803, 4802, 4811, 4807, 4806, 4815, 4810, 4819, 4818, 4814,
4823, 4822, 4827, 4826, 4835, 4831, 4830, 4839, 4834, 4843,
4842, 4838, 4847, 4846, 4851, 4850, 4859, 4855, 4854, 4863,
4858, 4867, 4866, 4862, 4871, 4870, 4875, 4874, 4883, 4879,
4878, 4887, 4882, 4891, 4890, 4886, 4895, 4894, 4899, 4898,
4907, 4903, 4902, 4911, 4906, 4915, 4914, 4910, 4919, 4918,
4923, 4922, 4931, 4927, 4926, 4935, 4930, 4939, 4938, 4934,
4943, 4942, 4947, 4946, 4955, 4951, 4950, 4959, 4954, 4963,
4962, 4958, 4967, 4966, 4971, 4970, 4979, 4975, 4974, 4983,
4978, 4987, 4986, 4982, 4991, 4990, 4995, 4994, 5003, 4999,
4998, 5007, 5002, 5011, 5010, 5006, 5015, 5014, 5019, 5018,
5027, 5023, 5022, 5031, 5026, 5035, 5034, 5030, 5039, 5038,
5043, 5042, 5051, 5047, 5046, 5055, 5050, 5059, 5058, 5054,
5063, 5062, 5067, 5066, 5075, 5071, 5070, 5079, 5074, 5083,
5082, 5078, 5087, 5086, 5091, 5090, 5099, 5095, 5094, 5103,
5098, 5107, 5106, 5102, 5111, 5110, 5115, 5114, 5123, 5119,
5118, 5127, 5122, 5131, 5130, 5126, 5135, 5134, 5139, 5138,
5147, 5143, 5142, 5151, 5146, 5155, 5154, 5150, 5159, 5158,
5163, 5162, 5171, 5167, 5166, 5175, 5170, 5179, 5178, 5174,
5183, 5182, 5187, 5186, 5195, 5191, 5190, 5199, 5194, 5203,
5202, 5198, 5207, 5206, 5211, 5210, 5219, 5215, 5214, 5223,
5218, 5227, 5226, 5222, 5231, 5230, 5235, 5234, 5243, 5239,
5238, 5247, 5242, 5251, 5250, 5246, 5255, 5254, 5259, 5258,
5267, 5263, 5262, 5271, 5266, 5275, 5274, 5270, 5279, 5278,
5283, 5282, 5291, 5287, 5286, 5295, 5290, 5299, 5298, 5294,
5303, 5302, 5307, 5306, 5315, 5311, 5310, 5319, 5314, 5323,
5322, 5318, 5327, 5326, 5331, 5330, 5339, 5335, 5334, 5343,
5338, 5347, 5346, 5342, 5351, 5350, 5355, 5354, 5363, 5359,
5358, 5367, 5362, 5371, 5370, 5366, 5375, 5374, 5379, 5378,
5387, 5383, 5382, 5391, 5386, 5395, 5394, 5390, 5399, 5398,
5403, 5402, 5411, 5407, 5406, 5415, 5410, 5419, 5418, 5414,
5423, 5422, 5427, 5426, 5435, 5431, 5430, 5439, 5434, 5443,
5442, 5438, 5447, 5446, 5451, 5450, 5459, 5455, 5454, 5463,
5458, 5467, 5466, 5462, 5471, 5470, 5475, 5474, 5483, 5479,
5478, 5487, 5482, 5491, 5490, 5486, 5495, 5494, 5499, 5498,
5507, 5503, 5502, 5511, 5506, 5515, 5514, 5510, 5519, 5518,
5523, 5522, 5531, 5527, 5526, 5535, 5530, 5539, 5538, 5534,
5543, 5542, 5547, 5546, 5555, 5551, 5550, 5559, 5554, 5563,
5562, 5558, 5567, 5566, 5571, 5570, 5579, 5575, 5574, 5583,
5578, 5587, 5586, 5582, 5591, 5590, 5595, 5594, 5603, 5599,
5598, 5607, 5602, 5611, 5610, 5606, 5615, 5614, 5619, 5618,
5627, 5623, 5622, 5631, 5626, 5635, 5634, 5630, 5639, 5638,
5643, 5642, 5651, 5647, 5646, 5655, 5650, 5659, 5658, 5654,
5663, 5662, 5667, 5666, 5675, 5671, 5670, 5679, 5674, 5683,
5682, 5678, 5687, 5686, 5691, 5690, 5699, 5695, 5694, 5703,
5698, 5707, 5706, 5702, 5711, 5710, 5715, 5714, 5723, 5719,
5718, 5727, 5722, 5731, 5730, 5726, 5735, 5734, 5739, 5738,
5747, 5743, 5742, 5751, 5746, 5755, 5754, 5750, 5759, 5758,
};

/*

                    GNU GENERAL PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

  0. This License applies to any program or other work which contains
a notice placed by the copyright holder saying it may be distributed
under the terms of this General Public License.  The "Program", below,
refers to any such program or work, and a "work based on the Program"
means either the Program or any derivative work under copyright law:
that is to say, a work containing the Program or a portion of it,
either verbatim or with modifications and/or translated into another
language.  (Hereinafter, translation is included without limitation in
the term "modification".)  Each licensee is addressed as "you".

Activities other than copying, distribution and modification are not
covered by this License; they are outside its scope.  The act of
running the Program is not restricted, and the output from the Program
is covered only if its contents constitute a work based on the
Program (independent of having been made by running the Program).
Whether that is true depends on what the Program does.

  1. You may copy and distribute verbatim copies of the Program's
source code as you receive it, in any medium, provided that you
conspicuously and appropriately publish on each copy an appropriate
copyright notice and disclaimer of warranty; keep intact all the
notices that refer to this License and to the absence of any warranty;
and give any other recipients of the Program a copy of this License
along with the Program.

You may charge a fee for the physical act of transferring a copy, and
you may at your option offer warranty protection in exchange for a fee.

  2. You may modify your copy or copies of the Program or any portion
of it, thus forming a work based on the Program, and copy and
distribute such modifications or work under the terms of Section 1
above, provided that you also meet all of these conditions:

    a) You must cause the modified files to carry prominent notices
    stating that you changed the files and the date of any change.

    b) You must cause any work that you distribute or publish, that in
    whole or in part contains or is derived from the Program or any
    part thereof, to be licensed as a whole at no charge to all third
    parties under the terms of this License.

    c) If the modified program normally reads commands interactively
    when run, you must cause it, when started running for such
    interactive use in the most ordinary way, to print or display an
    announcement including an appropriate copyright notice and a
    notice that there is no warranty (or else, saying that you provide
    a warranty) and that users may redistribute the program under
    these conditions, and telling the user how to view a copy of this
    License.  (Exception: if the Program itself is interactive but
    does not normally print such an announcement, your work based on
    the Program is not required to print an announcement.)

These requirements apply to the modified work as a whole.  If
identifiable sections of that work are not derived from the Program,
and can be reasonably considered independent and separate works in
themselves, then this License, and its terms, do not apply to those
sections when you distribute them as separate works.  But when you
distribute the same sections as part of a whole which is a work based
on the Program, the distribution of the whole must be on the terms of
this License, whose permissions for other licensees extend to the
entire whole, and thus to each and every part regardless of who wrote it.

Thus, it is not the intent of this section to claim rights or contest
your rights to work written entirely by you; rather, the intent is to
exercise the right to control the distribution of derivative or
collective works based on the Program.

In addition, mere aggregation of another work not based on the Program
with the Program (or with a work based on the Program) on a volume of
a storage or distribution medium does not bring the other work under
the scope of this License.

  3. You may copy and distribute the Program (or a work based on it,
under Section 2) in object code or executable form under the terms of
Sections 1 and 2 above provided that you also do one of the following:

    a) Accompany it with the complete corresponding machine-readable
    source code, which must be distributed under the terms of Sections
    1 and 2 above on a medium customarily used for software interchange; or,

    b) Accompany it with a written offer, valid for at least three
    years, to give any third party, for a charge no more than your
    cost of physically performing source distribution, a complete
    machine-readable copy of the corresponding source code, to be
    distributed under the terms of Sections 1 and 2 above on a medium
    customarily used for software interchange; or,

    c) Accompany it with the information you received as to the offer
    to distribute corresponding source code.  (This alternative is
    allowed only for noncommercial distribution and only if you
    received the program in object code or executable form with such
    an offer, in accord with Subsection b above.)

The source code for a work means the preferred form of the work for
making modifications to it.  For an executable work, complete source
code means all the source code for all modules it contains, plus any
associated interface definition files, plus the scripts used to
control compilation and installation of the executable.  However, as a
special exception, the source code distributed need not include
anything that is normally distributed (in either source or binary
form) with the major components (compiler, kernel, and so on) of the
operating system on which the executable runs, unless that component
itself accompanies the executable.

If distribution of executable or object code is made by offering
access to copy from a designated place, then offering equivalent
access to copy the source code from the same place counts as
distribution of the source code, even though third parties are not
compelled to copy the source along with the object code.

  4. You may not copy, modify, sublicense, or distribute the Program
except as expressly provided under this License.  Any attempt
otherwise to copy, modify, sublicense or distribute the Program is
void, and will automatically terminate your rights under this License.
However, parties who have received copies, or rights, from you under
this License will not have their licenses terminated so long as such
parties remain in full compliance.

  5. You are not required to accept this License, since you have not
signed it.  However, nothing else grants you permission to modify or
distribute the Program or its derivative works.  These actions are
prohibited by law if you do not accept this License.  Therefore, by
modifying or distributing the Program (or any work based on the
Program), you indicate your acceptance of this License to do so, and
all its terms and conditions for copying, distributing or modifying
the Program or works based on it.

  6. Each time you redistribute the Program (or any work based on the
Program), the recipient automatically receives a license from the
original licensor to copy, distribute or modify the Program subject to
these terms and conditions.  You may not impose any further
restrictions on the recipients' exercise of the rights granted herein.
You are not responsible for enforcing compliance by third parties to
this License.

  7. If, as a consequence of a court judgment or allegation of patent
infringement or for any other reason (not limited to patent issues),
conditions are imposed on you (whether by court order, agreement or
otherwise) that contradict the conditions of this License, they do not
excuse you from the conditions of this License.  If you cannot
distribute so as to satisfy simultaneously your obligations under this
License and any other pertinent obligations, then as a consequence you
may not distribute the Program at all.  For example, if a patent
license would not permit royalty-free redistribution of the Program by
all those who receive copies directly or indirectly through you, then
the only way you could satisfy both it and this License would be to
refrain entirely from distribution of the Program.

If any portion of this section is held invalid or unenforceable under
any particular circumstance, the balance of the section is intended to
apply and the section as a whole is intended to apply in other
circumstances.

It is not the purpose of this section to induce you to infringe any
patents or other property right claims or to contest validity of any
such claims; this section has the sole purpose of protecting the
integrity of the free software distribution system, which is
implemented by public license practices.  Many people have made
generous contributions to the wide range of software distributed
through that system in reliance on consistent application of that
system; it is up to the author/donor to decide if he or she is willing
to distribute software through any other system and a licensee cannot
impose that choice.

This section is intended to make thoroughly clear what is believed to
be a consequence of the rest of this License.

  8. If the distribution and/or use of the Program is restricted in
certain countries either by patents or by copyrighted interfaces, the
original copyright holder who places the Program under this License
may add an explicit geographical distribution limitation excluding
those countries, so that distribution is permitted only in or among
countries not thus excluded.  In such case, this License incorporates
the limitation as if written in the body of this License.

  9. The Free Software Foundation may publish revised and/or new versions
of the General Public License from time to time.  Such new versions will
be similar in spirit to the present version, but may differ in detail to
address new problems or concerns.

Each version is given a distinguishing version number.  If the Program
specifies a version number of this License which applies to it and "any
later version", you have the option of following the terms and conditions
either of that version or of any later version published by the Free
Software Foundation.  If the Program does not specify a version number of
this License, you may choose any version ever published by the Free Software
Foundation.

  10. If you wish to incorporate parts of the Program into other free
programs whose distribution conditions are different, write to the author
to ask for permission.  For software which is copyrighted by the Free
Software Foundation, write to the Free Software Foundation; we sometimes
make exceptions for this.  Our decision will be guided by the two goals
of preserving the free status of all derivatives of our free software and
of promoting the sharing and reuse of software generally.

                            NO WARRANTY

  11. BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY
FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW.  EXCEPT WHEN
OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES
PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED
OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE RISK AS
TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.  SHOULD THE
PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
REPAIR OR CORRECTION.

  12. IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING
WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR
REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES,
INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED
TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY
YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER
PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.

*/
