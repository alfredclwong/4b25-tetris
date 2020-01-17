#define COLS 10
#define ROWS 20
#define MARGIN 3
#define BUFFER 4
#define LOCK_MS 500
#define NEXT 4
#define NUM_PIECES 7
#define COL_SPAWN 4 // spawn on 4th (-1) to 7th (2) cols
#define ROW_SPAWN 20 // spawn on 21st (0) and 22nd (1) rows
#define FPS 120

void play();

/********************************************/
/*          CORE TETRIS                     */
/********************************************/
typedef struct {
    int8_t x, y;
} Point;

typedef struct {
		int id;
    Point points[4];
} Piece;

static const Piece PIECES[7] = {
    { .id = 0, .points = {{-1,  0}, { 0,  0}, { 1,  0}, {-1,  1}} }, // J
    { .id = 1, .points = {{-1,  0}, { 0,  0}, { 1,  0}, { 2,  0}} }, // I
    { .id = 2, .points = {{ 0,  0}, { 1,  0}, {-1,  1}, { 0,  1}} }, // Z
    { .id = 3, .points = {{-1,  0}, { 0,  0}, { 1,  0}, { 1,  1}} }, // L
    { .id = 4, .points = {{ 0,  0}, { 0,  1}, { 1,  0}, { 1,  1}} }, // O
    { .id = 5, .points = {{-1,  0}, { 0,  0}, { 1,  0}, { 0,  1}} }, // T
    { .id = 6, .points = {{-1,  0}, { 0,  0}, { 0,  1}, { 1,  1}} }, // S
};

/********************************************/
/*          aEsTheTicS                      */
/********************************************/
static const uint16_t color_masks[3] = {
    0b1111100000000000,
    0b0000011111100000,
    0b0000000000011111,
};
#define GREY 			0b0011100111100111
#define BLUE      0b0000000000011111
#define CYAN      0b0000011111111111
#define RED       0b1111100000000000
#define ORANGE    0b1111101111100000
#define YELLOW    0b1111111111100000
#define PURPLE    0b1111100000011111
#define GREEN     0b0000011111100000
#define WHITE			0b1111111111111111
static const uint16_t COLORS[] = {BLUE, CYAN, RED, ORANGE, YELLOW, PURPLE, GREEN};

static const Point SEGMENTS[7][2] = {
		{{0, 4}, {2, 4}},
		{{2, 4}, {2, 2}},
		{{2, 2}, {2, 0}},
		{{2, 0}, {0, 0}},
		{{0, 0}, {0, 2}},
		{{0, 2}, {0, 4}},
		{{0, 2}, {2, 2}},
};

static const uint8_t DIGITS[10] = {
		0b00111111, // MSB <- LSB
		0b00000110,
		0b01011011,
		0b01001111,
		0b01100110,
		0b01101101,
		0b01111101,
		0b00000111,
		0b01111111,
		0b01100111,
};
