#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include "tetris.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"
#include "fsl_os_abstraction_bm.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"
#include "devINA219.h"

/* Weak function. */
#if defined(__GNUC__)
#define __WEAK_FUNC __attribute__((weak))
#elif defined(__ICCARM__)
#define __WEAK_FUNC __weak
#elif defined( __CC_ARM )
#define __WEAK_FUNC __weak
#endif

volatile uint8_t inBuffer[1], payloadBytes[1];
bool held = 0, falling = 1, soft_dropping =0, done = 0, paused = 0;
Point loc;
Piece fall;
int hold = -1, next[NEXT], bag[NUM_PIECES], next_head, bag_head, level, lines_cleared = 0;
uint32_t prev_read, prev_draw, prev_fall, curr, lock_start;

static int
sendByte(uint8_t byte)
{
	spi_status_t status;
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);
	payloadBytes[0] = byte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);
	GPIO_DRV_SetPinOutput(kSSD1331PinDC);
	return status;
}

void fill_bag(int bag[NUM_PIECES]) {
    // generate random perm
    for (int i=0; i<NUM_PIECES; i++)
        bag[i] = i;

		/*
    for (int i=0; i<NUM_PIECES; i++) {
        int j, tmp;
        j = rand() % (NUM_PIECES-i) + i;
        tmp = perm[j];
        perm[j] = perm[i];
        perm[i] = tmp;
    }
		*/
}

int pop_next(int next[NEXT], int bag[NUM_PIECES], int *next_head, int *bag_head) {
		// require non-empty bag for repopulating next after pop
    if (*bag_head == NUM_PIECES)
        fill_bag(bag);
		*bag_head = 0;

    // pop from next for retval, pop from bag for next next
    int next_piece = next[*next_head];
		next[*next_head] = bag[*bag_head];
		*next_head = (*next_head + 1) % NEXT;
		*bag_head = *bag_head + 1;
    return next_piece;
}

// TODO change this to a void fn which can also perform the action (if legal) and incoporate rotations
int can_fall(Piece *fall, Point *loc, int8_t matrix[COLS][ROWS]) {
    int x, y;
    for (int i=0; i<4; i++) {
        y = loc->y + fall->points[i].y - 1;
        if (y > ROWS-1)
            continue;
        if (y < 0)
            return 0;
        x = loc->x + fall->points[i].x;
        if (matrix[x][y] > -1)
            return 0;
    }
    return 1;
}

void rotate(Piece *fall, int direction) { // direciton: 1 => ccw, -1 => cw
    int8_t *x, *y, old_x;
    for (int i=0; i<4; i++) {
        x = &(fall->points[i].x);
        y = &(fall->points[i].y);
        old_x = fall->points[i].x;
        *x = -direction * (*y);
        *y = +direction * old_x;
    }
}

int can_rotate(Piece *fall, Point *loc, int8_t matrix[COLS][ROWS], int direction) {
    int x, y;
    Piece tmp = *fall;
    rotate(&tmp, direction);
    for (int i=0; i<4; i++) {
        x = loc->x + tmp.points[i].x;
        y = loc->y + tmp.points[i].y;
        if (y < 0 || x < 0 || x >= COLS || matrix[x][y] > -1)
            return 0;
    }
    return 1;
}

double speed(int level, int soft_dropping) {
		if (soft_dropping)
        return pow(0.8 - level * 0.007, level) / 20.0;
    return pow(0.8 - level * 0.007, level);
}

void clear(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
		GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
		sendByte(kSSD1331CommandDRAWRECT);
		sendByte(y1);
		sendByte(x1);
		sendByte(y2);
		sendByte(x2);
		for (int i=0; i<6; i++)
				sendByte(0);
		GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
}

void
drawSquare(uint8_t x, uint8_t y, uint16_t color)
{
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
	sendByte(kSSD1331CommandDRAWRECT);
	sendByte(4*y++);
	sendByte(4*x++);
	sendByte(4*y-1);
	sendByte(4*x-1);
	//sendByte(7);
	//sendByte(7);
	//sendByte(7);
	sendByte((color&color_masks[0])>>10);
	sendByte((color&color_masks[1])>>5);
	sendByte((color&color_masks[2])<<1);
	sendByte((color&color_masks[0])>>10);
	sendByte((color&color_masks[1])>>5);
	sendByte((color&color_masks[2])<<1);
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
}

void erasePiece(Point *loc, Piece *p) {
		for (int i=0; i<4; i++)
				drawSquare(MARGIN+loc->x+p->points[i].x, loc->y+p->points[i].y, 0);
}

/*FUNCTION**********************************************************************
 * 
 * Function Name : OSA_TimeDiff
 * Description   : This function gets the difference between two time stamp,
 * time overflow is considered.
 *
 *END**************************************************************************/
__WEAK_FUNC uint32_t OSA_TimeDiff(uint32_t time_start, uint32_t time_end)
{
		if (time_end >= time_start)
		{
				return time_end - time_start;
		}
		else
		{
				/* lptmr count is 16 bits. */
				return FSL_OSA_TIME_RANGE - time_start + time_end + 1;
		}
}

void drawNumber(int number, int n_digits, uint8_t x, uint8_t y, uint16_t color) {
		int pow10 = 1, digit;
		for (int i=n_digits-1; i>=0; i--) {
				digit = number / pow10;
				pow10 *= 10;
				digit %= pow10;
				for (int j=0; j<7; j++) {
						if (DIGITS[digit] >> j & 1) {
								GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
								sendByte(kSSD1331CommandDRAWLINE);
								for (int k=0; k<2; k++) {
									sendByte(y+SEGMENTS[j][k].y);
									sendByte(x+SEGMENTS[j][k].x+i*4);
								}
								sendByte((color&color_masks[0])>>10);
								sendByte((color&color_masks[1])>>5);
								sendByte((color&color_masks[2])<<1);
								GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
						}
				}
		}
}

Point getGhostLoc(Piece *fall, Point *loc, int8_t matrix[COLS][ROWS]) {
		Point ghostLoc = *loc;
		while (can_fall(fall, &ghostLoc, matrix))
				ghostLoc.y--;
		return ghostLoc;
}

void play() {
		GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
		OSA_TimeDelay(10);

		/****************************************/
		/*						INA219 setup							*/
		/****************************************/
		initINA219(0x40, NULL);
		SEGGER_RTT_printf(0, "Idle current reading: %d\n", readCurrent());

		int8_t matrix[COLS][ROWS];
		for (int x=0; x<COLS; x++)
				for (int y=0; y<ROWS; y++)
						matrix[x][y] = -1;
		fill_bag(bag);
		for (int i=0; i<NEXT; i++)
				next[i] = bag[i];
		bag_head = NEXT;
		next_head = 0;
    
		//srand(time(NULL));

		level = 0;
		done = 0;
    while (!done) {
        /********************************************/
        /*              GENERATION PHASE            */
        /********************************************/
        loc.x = COL_SPAWN;
				loc.y = ROW_SPAWN;
        fall = PIECES[next[next_head]];
				if (bag_head == NUM_PIECES) {
						fill_bag(bag);
						bag_head = 0;
				}
				next[next_head] = bag[bag_head++];
				next_head = (next_head+1)%NEXT;
				clear(55, 0, 63, 95);

        /********************************************/
        /*              FALLING/LOCK PHASE          */
        /********************************************/
				held = 0;
				falling = 1;
				soft_dropping = 0;
        curr = OSA_TimeGetMsec();
				prev_fall = curr - 999;
        prev_draw = curr - 999;
				prev_read = curr;
        while (!done) {
						char key = SEGGER_RTT_GetKey();
						int dx, drot;
						switch(key) {
								/*
								case 'q':
										done = 1;
										continue;
								*/
								case 'p':
										paused = !paused;
										continue;
								case 'r':
										level = 0;
										lines_cleared = 0;
										for (int x=0; x<COLS; x++)
												for (int y=0; y<ROWS; y++)
														matrix[x][y] = -1;
										clear(12, 0, 53, 95); // matrix
										clear(0, 0, 11, 11); // level and lines
										break;
								case 'n': // left
								case 'm': // right
										dx = key == 'n' ? -1 : 1;
										for (int i=0; i<4; i++) {
												int x = loc.x + fall.points[i].x + dx,
														y = loc.y + fall.points[i].y;
												if (x<0 || x>=COLS || (y<ROWS && matrix[x][y]>-1)) {
														dx = 0;
														break;
												}
										}
										if (dx) {
											Point ghost_loc = getGhostLoc(&fall, &loc, matrix);
											erasePiece(&ghost_loc, &fall);
											erasePiece(&loc, &fall);
											loc.x += dx;
										}
										break;
								case ' ': // hard drop
										erasePiece(&loc, &fall);
										while (can_fall(&fall, &loc, matrix))
												loc.y--;
										falling = 0;
										lock_start = curr - 999;
										break;
								case ',': // soft drop
										soft_dropping = 1;
										break;
								case 'z': // rotate ccw
								case 'x': // rotate cw
										drot = key == 'z' ? 1: -1;
										if (can_rotate(&fall, &loc, matrix, drot)) {
												Point ghost_loc = getGhostLoc(&fall, &loc, matrix);
												erasePiece(&ghost_loc, &fall);
												erasePiece(&loc, &fall);
												rotate(&fall, drot);
										}
										break;
								case 'c': // hold
										{Point ghost_loc = getGhostLoc(&fall, &loc, matrix);
										erasePiece(&ghost_loc, &fall);}
										erasePiece(&loc, &fall);
										if (held)
												break;
										if (hold > -1) { // swap hold and fall
												int tmp = hold;
												hold = fall.id;
												fall = PIECES[tmp];
										} else { // nothing in hold
												hold = fall.id;
												fall = PIECES[next[next_head]];
												if (bag_head == NUM_PIECES) {
														fill_bag(bag);
														bag_head = 0;
												}
												next[next_head] = bag[bag_head++];
												next_head = (next_head+1)%NEXT;
												clear(55, 0, 63, 95); // clear next
										}
										loc.x = COL_SPAWN;
										loc.y = ROW_SPAWN;
										held = 1;
										clear(0, 0, 10, 95); // clear hold
										break;
								default:
										soft_dropping = 0;
										break;
						}

						curr = OSA_TimeGetMsec();

						if (OSA_TimeDiff(prev_read, curr) > 1000) {
								SEGGER_RTT_printf(0, "%d\n", readCurrent());
								prev_read = curr;
						}

						if (paused) // TODO maintaim time differences during pause
								continue;

            if (!falling) {
                // check lock and update lock timer
                if (OSA_TimeDiff(lock_start, curr) > LOCK_MS) {
                    for (int i=0; i<4; i++) {
                        int x = loc.x + fall.points[i].x;
                        int y = loc.y + fall.points[i].y;
                        matrix[x][y] = fall.id;
                    }
                    break; /* EXIT POINT */
                }
                if (can_fall(&fall, &loc, matrix))
                    falling = 1;
            }

            if (falling) { // note: do not use an else as the lock block above can change the value of falling
                if (OSA_TimeDiff(prev_fall, curr) > speed(level, soft_dropping) * 1000.0) {
                    if (can_fall(&fall, &loc, matrix)) {
												erasePiece(&loc, &fall);
                        loc.y--;
                        prev_fall = curr;
                    } else {
                        falling = 0;
												lock_start = OSA_TimeGetMsec();
                    }
                }
            }

            // redraw game screen 
            if (OSA_TimeDiff(prev_draw, curr) >= 1000.0 / FPS) {
								//clear(12, 0, 53, 95);
								//OSA_TimeDelay(1);

								// draw borders
								GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
								for (int i=0; i<2; i++) {
										sendByte(kSSD1331CommandDRAWLINE);
										sendByte(0);
										sendByte(i ? 11 : 52);
										sendByte(95);
										sendByte(i ? 11 : 52);
										sendByte(63);
										sendByte(63);
										sendByte(63);
								}
								GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

								// draw matrix
								for (int x=0; x<COLS; x++)
										for (int y=0; y<ROWS; y++)
												if (matrix[x][y] > -1)
														drawSquare(MARGIN+x, y, COLORS[matrix[x][y]]);

								// draw ghost (before fall)
								Point ghost_loc = getGhostLoc(&fall, &loc, matrix);
								for (int i=0; i<4; i++)
										drawSquare(MARGIN+ghost_loc.x+fall.points[i].x, ghost_loc.y+fall.points[i].y, GREY); // TODO fade colors

								// draw fall
								for (int i=0; i<4; i++)
										drawSquare(MARGIN+loc.x+fall.points[i].x, loc.y+fall.points[i].y, COLORS[fall.id]);

								Piece rot;

								// draw next
								for (int i=0; i<NEXT; i++) {
										rot = PIECES[*(next+(next_head+i)%NEXT)];
										rotate(&rot, -1);
										for (int j=0; j<4; j++)
												drawSquare(MARGIN+COLS+1+rot.points[j].x, ROWS-(5*i)+rot.points[j].y, COLORS[rot.id]);
								}

								// draw hold
								if (hold > -1) {
										rot = PIECES[hold];
										rotate(&rot, -1);
										for (int i=0; i<4; i++)
												drawSquare(rot.points[i].x, ROWS+rot.points[i].y, COLORS[hold]);
								}

								/*
								// draw bag
								for (int i=0; bag_head+i<NUM_PIECES && i<4; i++) {
										rot = PIECES[bag[bag_head+i]];
										rotate(&rot, -1);
										for (int j=0; j<4; j++)
												drawSquare(rot.points[j].x, ROWS-(5*i)+rot.points[j].y, COLORS[rot.id]);
								}
								*/

								// draw stats
								drawNumber(lines_cleared, 3, 0, 0, WHITE);
								drawNumber(level, 2, 0, 7, WHITE);
								clear(0, 20, 11, 24);
								drawNumber(1000 / (curr - prev_draw), 3, 0, 20, WHITE);

								prev_draw = OSA_TimeGetMsec();
            }
        }
        
        /********************************************/
        /*          PATTERN/ELIMINATION PHASE       */
        /********************************************/
				// pattern
        int lines[4] = {-1}, n_lines = 0;
        for (int y=0; y<ROWS; y++) {
            int complete = 1;
            for (int x=0; x<COLS; x++) {
                if (matrix[x][y] == -1) {
                    complete = 0;
                    break;
                }
            }
            if (complete) {
                lines[n_lines++] = y;
            }
        }

        // elimination
        for (int i=0; i<n_lines; i++) {
						clear(12, lines[i]*4, 53, 95);
            for (int y=lines[i]-i; y<ROWS-1; y++)
                for (int x=0; x<COLS; x++)
                    matrix[x][y] = matrix[x][y+1];
            for (int x=0; x<COLS; x++)
                matrix[x][ROWS-1] = -1;
        }
				if (n_lines) {
						clear(0, 0, 11, 4);
						lines_cleared += n_lines;
				}
				if (level < 15 && lines_cleared >= 10*(level+1)) {
						clear(0, 7, 11, 11);
						level++;
				}
    }
}
