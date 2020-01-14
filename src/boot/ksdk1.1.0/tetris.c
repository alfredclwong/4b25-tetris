#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <ncurses.h>
#include <math.h>
#include "tetris.h"

void fill_bag(LinkedPiece **bag) {
    // generate random perm
    int perm[NUM_PIECES];
    for (int i=0; i<NUM_PIECES; i++)
        perm[i] = i;

    for (int i=0; i<NUM_PIECES; i++) {
        int j, tmp;
        j = rand() % (NUM_PIECES-i) + i;
        tmp = perm[j];
        perm[j] = perm[i];
        perm[i] = tmp;
    }

    // fill bag according to perm
    *bag = (LinkedPiece*) malloc(sizeof(LinkedPiece));
    LinkedPiece *end = *bag;
    for (int i=0; i<NUM_PIECES-1; i++) {
        end->piece = PIECES[perm[i]];
        end->next = (LinkedPiece*) malloc(sizeof(LinkedPiece));
        end = end->next;
    }
    end->piece = PIECES[perm[NUM_PIECES-1]];
    end->next = NULL;
}

Piece pop_next(LinkedPiece **next, LinkedPiece **bag) {
    // require non-empty bag for repopulating next after pop
    if (*bag == NULL)
        fill_bag(bag);

    // edge case: first game loop has empty next
    if (*next == NULL) {
        *next = *bag;
        LinkedPiece *end = *next;
        for (int i=0; i<NEXT-1; i++)
            end = end->next;
        *bag = end->next;
        end->next = NULL;
    }

    // pop from next for retval, pop from bag for next next
    Piece next_piece = (*next)->piece;
    *next = (*next)->next;
    LinkedPiece *end = *next;
    while (end->next)
        end = end->next;
    end->next = *bag;
    *bag = (*bag)->next;
    end->next->next = NULL;
    return next_piece;
}

long usec(struct timeval *a) {
    return a->tv_sec*1000000 + a->tv_usec;
}

long usec_diff(struct timeval *a, struct timeval *b) {
    return usec(a) - usec(b);
}

// TODO change this to a void fn which can also perform the action (if legal) and incoporate rotations
int can_fall(Piece *fall, Point *loc, int matrix[COLS][ROWS]) {
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
    int *x, *y, old_x;
    for (int i=0; i<4; i++) {
        x = &(fall->points[i].x);
        y = &(fall->points[i].y);
        old_x = fall->points[i].x;
        *x = -direction * (*y);
        *y = +direction * old_x;
    }
}

int can_rotate(Piece *fall, Point *loc, int matrix[COLS][ROWS], int direction) {
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
        return pow(0.8 - level * 0.007, level) / 20.0;
    return pow(0.8 - level * 0.007, level);
}

void draw(int matrix[COLS][ROWS], Point loc, Piece *fall, Piece *hold, LinkedPiece *next, LinkedPiece *bag) {
    // draw matrix
    for (int x=0; x<COLS; x++)
        for (int y=0; y<ROWS; y++)
            if (matrix[x][y])
                mvprintw(BUFFER+ROWS-1-y, 5+x, "%d", matrix[x][y]);
    // draw ghost (before fall)
    Point ghost_loc = loc;
    while (can_fall(fall, &ghost_loc, matrix))
        ghost_loc.y--;
    for (int i=0; i<4; i++) {
        mvprintw(BUFFER+ROWS-1-(ghost_loc.y+fall->points[i].y), 5+ghost_loc.x+fall->points[i].x, "*");
    }
    // draw fall
    for (int i=0; i<4; i++)
        mvprintw(BUFFER+ROWS-1-(loc.y+fall->points[i].y), 5+loc.x+fall->points[i].x, "1");
    // draw next
    mvprintw(BUFFER-2, 5+COLS+1, "NEXT");
    LinkedPiece *end = next;
    for (int i=0; end; i++) {
        for (int j=0; j<4; j++)
            mvprintw(BUFFER+i*3+1-end->piece.points[j].y, 5+COLS+2+end->piece.points[j].x, "1");
        end = end->next;
    }
    // draw hold
    mvprintw(BUFFER-2, 0, "HOLD");
    if (hold != NULL) {
        for (int i=0; i<4; i++)
            mvprintw(BUFFER+1-hold->points[i].y, 1+hold->points[i].x, "1");
    }
    // draw bag
    mvprintw(BUFFER-2, 5+COLS+1+5, "BAG");
    end = bag;
    for (int i=0; end; i++) {
        for (int j=0; j<4; j++)
            mvprintw(BUFFER+i*3+1-end->piece.points[j].y, 5+COLS+2+5+end->piece.points[j].x, "1");
        end = end->next;
    }
}

int main() {
    srand(time(NULL));

    int matrix[COLS][ROWS] = {{0}};
    Point loc, spawn = {COL_SPAWN, ROW_SPAWN};
    Piece *fall = NULL, *hold = NULL;
    LinkedPiece *next = NULL, *bag = NULL;

    initscr();
    noecho();
    curs_set(0);
    timeout(0);
    keypad(stdscr, TRUE);

    int done = 0, level = 0;
    while (!done) {
        /********************************************/
        /*              GENERATION PHASE            */
        /********************************************/
        loc = spawn;
        Piece next_piece = pop_next(&next, &bag);
        fall = &next_piece;

        /********************************************/
        /*              FALLING/LOCK PHASE          */
        /********************************************/
        int c, held = 0, falling = 1, soft_dropping = 0;
        struct timeval prev_draw, prev_fall, curr, lock_start, game_start;
        gettimeofday(&game_start, NULL);
        prev_fall = game_start;
        prev_draw = game_start;
        prev_fall.tv_sec -= 999;
        prev_draw.tv_sec -= 999;
        while (!done) {
            // deal with inputs
            c = getch();
            switch (c) {
                int dx, direction;
                case 'q':
                    done = 1;
                    continue;
                case KEY_LEFT: // move left
                case KEY_RIGHT: // move right
                    dx = c == KEY_LEFT ? -1 : 1;
                    for (int i=0; i<4; i++) {
                        int x = loc.x + fall->points[i].x + dx, y = loc.y + fall->points[i].y;
                        if (x<0 || x>=COLS || (y<ROWS && matrix[x][y])) {
                            dx = 0;
                            break;
                        }
                    }
                    loc.x += dx;
                    draw(matrix, loc, fall, hold, next, bag);
                    break;
                case KEY_UP: // TODO hard drop
                    while (can_fall(fall, &loc, matrix))
                            loc.y--;
                    falling = 0;
                    lock_start = curr;
                    lock_start.tv_sec -= 999;
                    break;
                case KEY_DOWN:
                    soft_dropping = 1;
                    break;
                case 'z': // rotate ccw
                case 'x': // rotate cw
                    direction = c == 'z' ? 1 : -1;
                    if (can_rotate(fall, &loc, matrix, direction))
                        rotate(fall, direction);
                    break;
                case 'c': // hold
                    if (held)
                        break;
                    if (hold) {
                        Piece tmp = *hold;
                        *hold = *fall;
                        *fall = tmp;
                    } else {
                        hold = (Piece*) malloc(sizeof(Piece));
                        *hold = *fall;
                        Piece next_piece = pop_next(&next, &bag);
                        fall = &next_piece;
                    }
                    loc = spawn;
                    held = 1;
                    draw(matrix, loc, fall, hold, next, bag);
                    break;
                default:
                    soft_dropping = 0; // TODO detect key release instead
                    break;
            }

            gettimeofday(&curr, NULL);

            if (!falling) {
                // check lock and update lock timer
                if (usec_diff(&curr, &lock_start) > LOCK_US) {
                    for (int i=0; i<4; i++) {
                        int x = loc.x + fall->points[i].x;
                        int y = loc.y + fall->points[i].y;
                        matrix[x][y] = 1;
                    }
                    //fall = NULL;
                    break; /* EXIT POINT */
                }
                if (can_fall(fall, &loc, matrix))
                    falling = 1;
            }

            if (falling) { // note: do not use an else as the lock block above can change the value of falling
                if (usec_diff(&curr, &prev_fall) > 1000000.0*speed(level, soft_dropping)) {
                    if (can_fall(fall, &loc, matrix)) {
                        loc.y--;
                        prev_fall = curr;
                    } else {
                        falling = 0;
                        gettimeofday(&lock_start, NULL);
                    }
                }
            }

            // redraw game screen 
            if (usec_diff(&curr, &prev_draw) > 1000000.0/FPS) {
                clear();
                draw(matrix, loc, fall, hold, next, bag);
                for (int i=0; i<4; i++) {
                    mvprintw(BUFFER+ROWS+1+i, 0, "(%d,%d)", fall->points[i].x, fall->points[i].y);
                    mvprintw(BUFFER+ROWS+1+i, 12, "(%d,%d)", loc.x+fall->points[i].x, loc.y+fall->points[i].y-1);
                    mvprintw(BUFFER+ROWS+1+i, 8, "%d", matrix[loc.x+fall->points[i].x][loc.y+fall->points[i].y-1]);
                }
                if (!falling)
                    mvprintw(BUFFER-2, 5+COL_SPAWN-1, "%.3f", (LOCK_US-usec_diff(&curr, &lock_start))/1000000.0);
                refresh();
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

        fall = NULL;
    }
    endwin();
}
