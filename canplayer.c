/*
 * canplayer.c - replay a compact CAN frame logfile to CAN devices
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <libgen.h>
#include <stdlib.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

#define DEFAULT_GAP	1	/* ms */
#define DEFAULT_LOOPS	1	/* only one replay */
#define CHANNELS	20	/* anyone using more than 20 CAN interfaces at a time? */
#define COMMENTSZ 200
#define BUFSZ (sizeof("(1345212884.318850)") + IFNAMSIZ + 4 + CL_CFSZ + COMMENTSZ) /* for one line in the logfile */
#define STDOUTIDX	65536	/* interface index for printing on stdout - bigger than max uint16 */

struct assignment {
	char txif[IFNAMSIZ];
	int  txifidx;
	char rxif[IFNAMSIZ];
};
static struct assignment asgn[CHANNELS];
const int canfd_on = 1;

extern int optind, opterr, optopt;

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s <options> [interface assignment]*\n\n", prg);
	fprintf(stderr, "Options:              -I <infile>  (default stdin)\n");
	fprintf(stderr, "                      -l <num>     "
		"(process input file <num> times)\n"
		"                                   "
		"(Use 'i' for infinite loop - default: %d)\n", DEFAULT_LOOPS);
	fprintf(stderr, "                      -t           (ignore timestamps: "
		"send frames immediately)\n");
	fprintf(stderr, "                      -g <ms>      (gap in milli "
		"seconds - default: %d ms)\n", DEFAULT_GAP);
	fprintf(stderr, "                      -s <s>       (skip gaps in "
		"timestamps > 's' seconds)\n");
	fprintf(stderr, "                      -x           (disable local "
		"loopback of sent CAN frames)\n");
	fprintf(stderr, "                      -v           (verbose: print "
		"sent CAN frames)\n\n");
	fprintf(stderr, "Interface assignment:  0..n assignments like "
		"<write-if>=<log-if>\n");
	fprintf(stderr, "e.g. vcan2=can0 ( send frames received from can0 on "
		"vcan2 )\n");
	fprintf(stderr, "extra hook: stdout=can0 ( print logfile line marked with can0 on "
		"stdout )\n");
	fprintf(stderr, "No assignments => send frames to the interface(s) they "
		"had been received from.\n\n");
	fprintf(stderr, "Lines in the logfile not beginning with '(' (start of "
		"timestamp) are ignored.\n\n");
}

static void ts_add(struct timespec *ts1, struct timespec *ts2, struct timespec *result)
{
	result->tv_sec  = ts1->tv_sec  + ts2->tv_sec;
	result->tv_nsec = ts1->tv_nsec + ts2->tv_nsec;
	if (result->tv_nsec >= 1000000000L) {
		result->tv_sec++ ;  result->tv_nsec = result->tv_nsec - 1000000000L;
	}
}

static void ts_subtract(struct timespec *ts1, struct timespec *ts2, struct timespec *result)
{
	if ( (ts1->tv_sec < ts2->tv_sec) ||
		( (ts1->tv_sec  == ts2->tv_sec) &&
		  (ts1->tv_nsec <= ts2->tv_nsec) ) ) {
		result->tv_sec = result->tv_nsec = 0;
	} else {
		result->tv_sec = ts1->tv_sec - ts2->tv_sec;

		if (ts1->tv_nsec < ts2->tv_nsec) {
			result->tv_nsec = ts1->tv_nsec + 1000000000L - ts2->tv_nsec;
			result->tv_sec--;
		} else {
			result->tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
		}
	}
}

int get_txidx(char *logif_name) {

	int i;

	for (i=0; i<CHANNELS; i++) {
		if (asgn[i].rxif[0] == 0) /* end of table content */
			break;
		if (strcmp(asgn[i].rxif, logif_name) == 0) /* found device name */
			break;
	}

	if ((i == CHANNELS) || (asgn[i].rxif[0] == 0))
		return 0; /* not found */

	return asgn[i].txifidx; /* return interface index */
}

char *get_txname(char *logif_name) {

	int i;

	for (i=0; i<CHANNELS; i++) {
		if (asgn[i].rxif[0] == 0) /* end of table content */
			break;
		if (strcmp(asgn[i].rxif, logif_name) == 0) /* found device name */
			break;
	}

	if ((i == CHANNELS) || (asgn[i].rxif[0] == 0))
		return 0; /* not found */

	return asgn[i].txif; /* return interface name */
}

int add_assignment(char *mode, int socket, char *txname, char *rxname,
		   int verbose) {

	struct ifreq ifr;
	int i;

	/* find free entry */
	for (i=0; i<CHANNELS; i++) {
		if (asgn[i].txif[0] == 0)
			break;
	}

	if (i == CHANNELS) {
		fprintf(stderr, "Assignment table exceeded!\n");
		return 1;
	}

	if (strlen(txname) >= IFNAMSIZ) {
		fprintf(stderr, "write-if interface name '%s' too long!", txname);
		return 1;
	}
	strcpy(asgn[i].txif, txname);

	if (strlen(rxname) >= IFNAMSIZ) {
		fprintf(stderr, "log-if interface name '%s' too long!", rxname);
		return 1;
	}
	strcpy(asgn[i].rxif, rxname);

	if (strcmp(txname, "stdout")) {
		strcpy(ifr.ifr_name, txname);
		if (ioctl(socket, SIOCGIFINDEX, &ifr) < 0) {
			perror("SIOCGIFINDEX");
			fprintf(stderr, "write-if interface name '%s' is wrong!\n", txname);
			return 1;
		}
		asgn[i].txifidx = ifr.ifr_ifindex;
	} else
		asgn[i].txifidx = STDOUTIDX;

	if (verbose > 1) /* use -v -v to see this */
		printf("added %s assignment: log-if=%s write-if=%s write-if-idx=%d\n",
		       mode, asgn[i].rxif, asgn[i].txif, asgn[i].txifidx);

	return 0;
}

int main(int argc, char **argv)
{
	static char buf[BUFSZ], device[BUFSZ], ascframe[BUFSZ];
	struct sockaddr_can addr;
	static struct canfd_frame frame;
	static struct timespec target_ts, log_ts, last_log_ts, diff_ts, sleep_ts;
	int s; /* CAN_RAW socket */
	FILE *infile = stdin;
	unsigned long gap = DEFAULT_GAP; 
	int use_timestamps = 1;
	static int verbose, opt, delay_loops;
	static unsigned long skipgap;
	static int loopback_disable = 0;
	static int infinite_loops = 0;
	static int loops = DEFAULT_LOOPS;
	int assignments; /* assignments defined on the commandline */
	int txidx;       /* sendto() interface index */
	int eof, txmtu, i, j;
	int err;
	char *fret;

	while ((opt = getopt(argc, argv, "I:l:tg:s:xv?")) != -1) {
		switch (opt) {
		case 'I':
			infile = fopen(optarg, "r");
			if (!infile) {
				perror("infile");
				return 1;
			}
			break;

		case 'l':
			if (optarg[0] == 'i')
				infinite_loops = 1;
			else
				if (!(loops = atoi(optarg))) {
					fprintf(stderr, "Invalid argument for option -l !\n");
					return 1;
				}
			break;

		case 't':
			use_timestamps = 0;
			break;

		case 'g':
			gap = strtoul(optarg, NULL, 10);
			break;

		case 's':
			skipgap = strtoul(optarg, NULL, 10);
			if (skipgap < 1) {
				fprintf(stderr, "Invalid argument for option -s !\n");
				return 1;
			}
			break;

		case 'x':
			loopback_disable = 1;
			break;

		case 'v':
			verbose++;
			break;

		case '?':
		default:
			print_usage(basename(argv[0]));
			return 1;
			break;
		}
	}

	assignments = argc - optind; /* find real number of user assignments */

	if (infile == stdin) { /* no jokes with stdin */
		infinite_loops = 0;
		loops = 1;
	}

	if (verbose > 1) { /* use -v -v to see this */
		if (infinite_loops)
			printf("infinite_loops\n");
		else
			printf("%d loops\n", loops);
	}

	sleep_ts.tv_sec  =  gap / 1000;
	sleep_ts.tv_nsec = (gap % 1000) * 1000000;

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family  = AF_CAN;
	addr.can_ifindex = 0;

	/* disable unneeded default receive filter on this RAW socket */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* try to switch the socket into CAN FD mode */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (loopback_disable) {
		int loopback = 0;

		setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
			   &loopback, sizeof(loopback));
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	if (assignments) {
		/* add & check user assginments from commandline */
		for (i=0; i<assignments; i++) {
			if (strlen(argv[optind+i]) >= BUFSZ) {
				fprintf(stderr, "Assignment too long!\n");
				print_usage(basename(argv[0]));
				return 1;
			}
			strcpy(buf, argv[optind+i]);
			for (j=0; j<BUFSZ; j++) { /* find '=' in assignment */
				if (buf[j] == '=')
					break;
			}
			if ((j == BUFSZ) || (buf[j] != '=')) {
				fprintf(stderr, "'=' missing in assignment!\n");
				print_usage(basename(argv[0]));
				return 1;
			}
			buf[j] = 0; /* cut string in two pieces */
			if (add_assignment("user", s, &buf[0], &buf[j+1], verbose))
				return 1;
		}
	}

	//special value of last log timestamp
	last_log_ts.tv_sec = last_log_ts.tv_nsec = -1;

	if (clock_gettime(CLOCK_MONOTONIC, &target_ts)) {
		fprintf(stderr, "CLOCK_MONOTONIC get failed\n");
		return 1;
	}

	while (infinite_loops || loops--) {

		if (infile != stdin)
			rewind(infile); /* for each loop */

		if (verbose > 1) /* use -v -v to see this */
			printf (">>>>>>>>> start reading file. remaining loops = %d\n", loops);

		/* read first non-comment frame from logfile */
		while ((fret = fgets(buf, BUFSZ-1, infile)) != NULL && buf[0] != '(') {
			if (strlen(buf) >= BUFSZ-2) {
				fprintf(stderr, "comment line too long for input buffer\n");
				return 1;
			}
		}

		if (!fret)
			goto out; /* nothing to read */

		eof = 0;

		if (sscanf(buf, "(%ld.%ld) %s %s", &log_ts.tv_sec, &log_ts.tv_nsec,
			   device, ascframe) != 4) {
			fprintf(stderr, "incorrect line format in logfile\n");
			return 1;
		}
		log_ts.tv_nsec *= 1000L; // convert usec to nsec

		if (use_timestamps) { /* throttle sending due to logfile timestamps */
			/* test for logfile timestamps jumping backwards OR      */
			/* if the user likes to skip long gaps in the timestamps */
			if ((last_log_ts.tv_sec == -1 && last_log_ts.tv_nsec == -1) ||
			    (last_log_ts.tv_sec > log_ts.tv_sec) ||
			    (skipgap && labs(last_log_ts.tv_sec - log_ts.tv_sec) > skipgap)) {
				diff_ts.tv_sec = diff_ts.tv_nsec = 0;
			} else {
				ts_subtract(&log_ts, &last_log_ts, &diff_ts);
			}

			last_log_ts = log_ts;

			ts_add(&target_ts, &diff_ts, &target_ts);
		} else {
			ts_add(&target_ts, &sleep_ts, &target_ts);
		}

		while (!eof) {
			while ((err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target_ts, NULL))) {
				if (err != EINTR) {
					fprintf(stderr, "sleep TIMER_ABSTIME failed %d\n", err);
					return 1;
				}
			}

			/* log_tv/device/ascframe are valid here */
			if (strlen(device) >= IFNAMSIZ) {
				fprintf(stderr, "log interface name '%s' too long!", device);
				return 1;
			}

			txidx = get_txidx(device); /* get ifindex for sending the frame */

			if ((!txidx) && (!assignments)) {
				/* ifindex not found and no user assignments */
				/* => assign this device automatically       */
				if (add_assignment("auto", s, device, device, verbose))
					return 1;
				txidx = get_txidx(device);
			}

			if (txidx == STDOUTIDX) { /* hook to print logfile lines on stdout */

				printf("%s", buf); /* print the line AS-IS without extra \n */
				fflush(stdout);

			} else if (txidx > 0) { /* only send to valid CAN devices */

				txmtu = parse_canframe(ascframe, &frame);
				if (!txmtu) {
					fprintf(stderr, "wrong CAN frame format: '%s'!", ascframe);
					return 1;
				}

				addr.can_family  = AF_CAN;
				addr.can_ifindex = txidx; /* send via this interface */

				if (sendto(s, &frame, txmtu, 0,	(struct sockaddr*)&addr, sizeof(addr)) != txmtu) {
					perror("sendto");
					return 1;
				}

				if (verbose) {
					printf("%s (%s) ", get_txname(device), device);

					if (txmtu == CAN_MTU)
						fprint_long_canframe(stdout, &frame, "", CANLIB_VIEW_INDENT_SFF, CAN_MAX_DLEN);
					else
						fprint_long_canframe(stdout, &frame, "", CANLIB_VIEW_INDENT_SFF, CANFD_MAX_DLEN);

					printf(" #+(%lld.%.9ld)\n", (long long)diff_ts.tv_sec, diff_ts.tv_nsec);
				}
			}

			/* read next non-comment frame from logfile */
			while ((fret = fgets(buf, BUFSZ-1, infile)) != NULL && buf[0] != '(') {
				if (strlen(buf) >= BUFSZ-2) {
					fprintf(stderr, "comment line too long for input buffer\n");
					return 1;
				}
			}

			if (!fret) {
				eof = 1; /* this file is completely processed */
				break;
			}

			if (sscanf(buf, "(%ld.%ld) %s %s", &log_ts.tv_sec, &log_ts.tv_nsec,
				   device, ascframe) != 4) {
				fprintf(stderr, "incorrect line format in logfile\n");
				return 1;
			}

			if (use_timestamps) {
				/* test for logfile timestamps jumping backwards OR      */
				/* if the user likes to skip long gaps in the timestamps */
				if ((last_log_ts.tv_sec > log_ts.tv_sec) ||
				    (skipgap && labs(last_log_ts.tv_sec - log_ts.tv_sec) > skipgap)) {
					diff_ts.tv_sec = diff_ts.tv_nsec = 0;
				} else {
					ts_subtract(&log_ts, &last_log_ts, &diff_ts);
				}

				last_log_ts = log_ts;

				ts_add(&target_ts, &diff_ts, &target_ts);
			} else {
				ts_add(&target_ts, &sleep_ts, &target_ts);
			}

			delay_loops++; /* private statistics */

		} /* while (!eof) */

	} /* while (infinite_loops || loops--) */

out:

	close(s);
	fclose(infile);

	if (verbose > 1) /* use -v -v to see this */
		printf("%d delay_loops\n", delay_loops);

	return 0;
}
