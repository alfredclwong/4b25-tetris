#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "tetris.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"
#include "fsl_os_abstraction_bm.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

/* Weak function. */
#if defined(__GNUC__)
#define __WEAK_FUNC __attribute__((weak))
#elif defined(__ICCARM__)
#define __WEAK_FUNC __weak
#elif defined( __CC_ARM )
#define __WEAK_FUNC __weak
#endif

volatile uint8_t inBuffer[1];
volatile uint8_t payloadBytes[1];

uint16_t color_masks[3] = {
	0b1111100000000000,
	0b0000011111100000,
	0b0000000000011111,
};

uint16_t purple = 0b1111100000011111;

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

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

void fill_bag(Piece *bag) {
    // generate random perm
    uint8_t perm[NUM_PIECES];
    for (int i=0; i<NUM_PIECES; i++)
        perm[i] = i;

		/*
    for (int i=0; i<NUM_PIECES; i++) {
        int j, tmp;
        j = rand() % (NUM_PIECES-i) + i;
        tmp = perm[j];
        perm[j] = perm[i];
        perm[i] = tmp;
    }
		*/

    // fill bag according to perm
    for (int i=0; i<NUM_PIECES; i++) {
        *(bag+i) = PIECES[perm[i]];
    }
}

Piece pop_next(Piece *next, Piece *bag, uint8_t *next_head, uint8_t *bag_head) {
		// require non-empty bag for repopulating next after pop
    if (*bag_head == NUM_PIECES)
        fill_bag(bag);
		*bag_head = 0;

    // pop from next for retval, pop from bag for next next
    Piece next_piece = *(next+(*next_head));
		*(next+(*next_head)) = *(bag+(*bag_head)++);
		*next_head = (*next_head + 1) % NEXT;
    return next_piece;
}

// TODO change this to a void fn which can also perform the action (if legal) and incoporate rotations
int can_fall(Piece *fall, Point *loc, bool matrix[COLS][ROWS]) {
    int x, y;
    for (int i=0; i<4; i++) {
        y = loc->y + fall->points[i].y - 1;
        if (y > ROWS-1)
            continue;
        if (y < 0)
            return 0;
        x = loc->x + fall->points[i].x;
        if (matrix[x][y])
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

int can_rotate(Piece *fall, Point *loc, bool matrix[COLS][ROWS], int direction) {
    int x, y;
    Piece tmp = *fall;
    rotate(&tmp, direction);
    for (int i=0; i<4; i++) {
        x = loc->x + tmp.points[i].x;
        y = loc->y + tmp.points[i].y;
        if (y < 0 || x < 0 || x >= COLS || matrix[x][y])
            return 0;
    }
    return 1;
}

double speed(int level, int soft_dropping) {
		if (soft_dropping)
        return 0.05;//pow(0.8 - level * 0.007, level) / 20.0;
    return 1;//pow(0.8 - level * 0.007, level);
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
	for (int i=0; i<2; i++) {
		sendByte((color&color_masks[0])>>10);
		sendByte((color&color_masks[1])>>5);
		sendByte((color&color_masks[2])<<1);
	}
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
}

void draw(bool matrix[COLS][ROWS], Point *loc, Piece *fall, Piece *hold, Piece *next, Piece *bag, uint8_t next_head, uint8_t bag_head, bool hold_empty) {
		/*
		 *	Clear Screen
		 */
		GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
		sendByte(kSSD1331CommandDRAWRECT);
		sendByte(0x00);
		sendByte(0x00);
		sendByte(0x5F);
		sendByte(0x3F);
		sendByte(0);
		sendByte(0);
		sendByte(0);
		sendByte(0);
		sendByte(0);
		sendByte(0);
		GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
		OSA_TimeDelay(1);
		GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
		// draw borders
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
            if (matrix[x][y])
                drawSquare(MARGIN+x, y, purple);
    
		// draw ghost (before fall)
    Point ghost_loc = *loc;
    while (can_fall(fall, &ghost_loc, matrix))
        ghost_loc.y--;
    for (int i=0; i<4; i++) {
        drawSquare(MARGIN+ghost_loc.x+fall->points[i].x, ghost_loc.y+fall->points[i].y, 0b0011100011100111);
    }

    // draw fall
    for (int i=0; i<4; i++)
        drawSquare(MARGIN+loc->x+fall->points[i].x, loc->y+fall->points[i].y, purple);

		Piece rot;

		// draw next
    //mvprintw(BUFFER-2, 5+COLS+1, "NEXT");
    for (int i=0; i<NEXT; i++) {
        rot = *(next + (next_head+i)%NEXT);
				rotate(&rot, -1);
				for (int j=0; j<4; j++)
            drawSquare(MARGIN+COLS+1+rot.points[j].x, ROWS-(5*i)+rot.points[j].y, purple);
    }

    // draw hold
    //mvprintw(BUFFER-2, 0, "HOLD");
    if (!hold_empty) {
        for (int i=0; i<4; i++)
            drawSquare(hold->points[i].x, ROWS-4+hold->points[i].y, purple);
    }
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

bool hold_empty = 1;
Point loc;
Piece fall, hold, next[NEXT], bag[NUM_PIECES];
uint8_t next_head, bag_head;

uint8_t done, level;
uint32_t prev_draw, prev_fall, curr, lock_start;

uint8_t held, falling, soft_dropping;
void play() {
		GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
		OSA_TimeDelay(10);

		bool matrix[COLS][ROWS] = {{0}};
		fill_bag(&bag[0]);
		for (int i=0; i<NEXT; i++) {
				next[i] = bag[i];
		}
		bag_head = NEXT;
		next_head = 0;
    
		//srand(time(NULL));

		level = 0;
		soft_dropping = 0;
		done = 0;

    while (!done) {
        /********************************************/
        /*              GENERATION PHASE            */
        /********************************************/
        loc.x = COL_SPAWN;
				loc.y = ROW_SPAWN;
        fall = pop_next(&next[0], &bag[0], &next_head, &bag_head);

        /********************************************/
        /*              FALLING/LOCK PHASE          */
        /********************************************/
				held = 0;
				falling = 1;
        curr = OSA_TimeGetMsec();
				prev_fall = 0;
        prev_draw = 0;
        while (!done) {
						// TODO deal with inputs

						curr = OSA_TimeGetMsec();

            if (!falling) {
                // check lock and update lock timer
                if (OSA_TimeDiff(lock_start, curr) > LOCK_MS) {
                    for (int i=0; i<4; i++) {
                        int x = loc.x + fall.points[i].x;
                        int y = loc.y + fall.points[i].y;
                        matrix[x][y] = 1;
                    }
                    break; /* EXIT POINT */
                }
                if (can_fall(&fall, &loc, matrix))
                    falling = 1;
            }

            if (falling) { // note: do not use an else as the lock block above can change the value of falling
                if (OSA_TimeDiff(prev_fall, curr) > speed(level, soft_dropping) * 1000.0) {
                    if (can_fall(&fall, &loc, matrix)) {
                        loc.y--;
                        prev_fall = curr;
                    } else {
                        falling = 0;
												lock_start = OSA_TimeGetMsec();
                    }
                }
            }

            // redraw game screen 
            if (OSA_TimeDiff(prev_draw, curr) > 1000.0 / FPS) {
                draw(matrix, &loc, &fall, &hold, &next[0], &bag[0], next_head, bag_head, hold_empty);
                prev_draw = curr;
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
                if (matrix[x][y] == 0) {
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
            for (int y=lines[i]-i; y<ROWS-1; y++)
                for (int x=0; x<COLS; x++)
                    matrix[x][y] = matrix[x][y+1];
            for (int x=0; x<COLS; x++)
                matrix[x][ROWS-1] = 0;
        }
    }
}
