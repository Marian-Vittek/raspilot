

// g++ -lGL -lGLU -lglut viz.cpp  && (ncat -l 3333 | ./a.out)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <cmath>

#include "linmath.h"

#define DIM(x)                          	(sizeof(x) / sizeof(x[0]))

///////////////////////////////////////////////////////////////////////////

#define CHECKB_WIDTH 	20
#define CHECKB_HEIGHT	20

// Colors
GLfloat WHITE[] = {1, 1, 1};
GLfloat RED[] = {1, 0, 0};
GLfloat GREEN[] = {0, 1, 0};
GLfloat BLUE[] = {0.5, 0.5, 1};
GLfloat MAGENTA[] = {1, 0, 1};
GLfloat GREY[] = {0.5, 0.5, 0.5};


////////////////////////////////////////////////////////////////////////

// A camera.  It moves horizontally in a circle centered at the origin of
// radius 10.  It moves vertically straight up and down.
class Camera {
  double x,y,z;
public:
  Camera(): x(CHECKB_WIDTH/2), y(CHECKB_HEIGHT/2), z(50) {}
  double getX() {return x;}
  double getY() {return y;}
  double getZ() {return z;}
  void moveRight() { x += z/20;}
  void moveLeft() { x -= z/20;}
  void moveUp() {y += z/20;}
  void moveDown() {y -= z/20;}
  void moveDxy(double dx, double dy) {x += dx*z/20; y += dy*z/20; }
  void zoomIn() {z -= 1;}
  void zoomOut() {z += 1;}
};

void wireBox(GLdouble width, GLdouble height, GLdouble depth) {
  glPushMatrix();
  glScalef(width, height, depth);
  glutWireCube(1.0);
  glPopMatrix();
}

GLUquadricObj *quadratic = gluNewQuadric();

class Arrows {
  double radius;
  double x;
  double y;
  double z;
  double xoffset;
  double yoffset;
  double zoffset;
  int direction;
  double q[4];	// x,y,z,w
  int allsetFlag;
  
public:
  Arrows(double x, double y, double z): xoffset(x), yoffset(y), zoffset(z), x(x), y(y), z(z) {
    radius = 0.1;
    memset(q, 0, sizeof(q));
    q[3] = 1.0;
    allsetFlag = 0;
  }
  void setQuat(double *qq) {	
	for(int i=0; i<4; i++) q[i] = qq[i];
	allsetFlag |= 0x01;
  }
  void setPose(double xx, double yy, double zz) {
    x = xx + xoffset;
    y = yy + yoffset;
    z = zz + zoffset;
    allsetFlag |= 0x02;
    // printf("pose --> %f %f %f\n", x, y, z);
  }
  void update() {
    if (allsetFlag == 0x0) return;
    glPushMatrix();
	{
	  glTranslated(x, y, z);
	  
	  // glMultMatrixd(quaternionToMatrix(q));
	  double qx = q[0];
	  double qy = q[1];
	  double qz = q[2];
	  double qw = q[3];
	  double mm[4][4] = {
		1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw, 0.0f,
		2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw, 0.0f,
		2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	  };
	  double mmt[4][4];
	  mat4x4_transpose(mmt, mm);
	  glMultMatrixd((const GLdouble*)mmt);

#if 1
	  // a simple "plane like" display from the top downward
	  // z
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, GREEN);
	  //gluCylinder(quadratic, radius, radius, 1, 4, 4);
	  //glTranslated(0, 0, 1);
	  gluCylinder(quadratic, 0.2f, 0.1f, 0.5, 16, 16);
	  //glTranslated(0, 0, -1);
	  // x
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, BLUE);
	  glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	  gluCylinder(quadratic, radius, radius, 3, 4, 4);
	  glTranslated(0, 0, 3);
	  gluCylinder(quadratic, 0.2f, 0.01f, 0.5, 16, 16);
	  glTranslated(0, 0, -3);
	  // y
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, RED);
	  // glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
	  glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	  glTranslated(0, -2, -2);
	  gluCylinder(quadratic, radius, radius, 4, 4, 4);
	  glTranslated(0, 2, 2);
	  //gluCylinder(quadratic, 0.2f, 0.01f, 0.5, 16, 16);
#else
	  // z
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, GREEN);
	  gluCylinder(quadratic, radius, radius, 1, 4, 4);
	  glTranslated(0, 0, 1);
	  gluCylinder(quadratic, 0.2f, 0.01f, 0.5, 16, 16);
	  glTranslated(0, 0, -1);
	  // x
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, BLUE);
	  glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	  gluCylinder(quadratic, radius, radius, 1, 4, 4);
	  glTranslated(0, 0, 1);
	  gluCylinder(quadratic, 0.2f, 0.01f, 0.5, 16, 16);
	  glTranslated(0, 0, -1);
	  // y
	  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, RED);
	  // glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
	  glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	  gluCylinder(quadratic, radius, radius, 1, 4, 4);
	  glTranslated(0, 0, 1);
	  gluCylinder(quadratic, 0.2f, 0.01f, 0.5, 16, 16);
#endif
	  
	}
    glPopMatrix();
  }
};

// A checkerboard class.  A checkerboard has alternating red and white
// squares.  The number of squares is set in the constructor.  Each square
// is 1 x 1.  One corner of the board is (0, 0) and the board stretches out
// along positive x and positive z.  It rests on the xz plane.  I put a
// spotlight at (4, 3, 7).
class Checkerboard {
  int displayListId;
  int width;
  int height;
public:
  Checkerboard(int width, int height): width(width), height(height) {}
  double centerx() {return width / 2;}
  double centery() {return height / 2;}
  double centerz() {return 0;}
  void create() {
    displayListId = glGenLists(1);
    glNewList(displayListId, GL_COMPILE);
    GLfloat lightPosition[] = {-20, 10, 10, 0.5};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glBegin(GL_QUADS);
    glNormal3d(0, 1, 0);
    /*
    for (int x = 0; x < width - 1; x++) {
      for (int y = 0; y < height - 1; y++) {
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
                     (x + y) % 2 == 0 ? RED : WHITE);
        glVertex3d(x, y, 0);
        glVertex3d(x+1, y, 0);
        glVertex3d(x+1, y+1, 0);
        glVertex3d(x, y+1, 0);

      }
    }
    */
    // high skeleton
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, GREY);
    for (int x = 0; x < width - 1; x++) {
      glVertex3d(x, height, 20);
      glVertex3d(x+0.05, height, 20);
      glVertex3d(x+0.05, 0, 20);
      glVertex3d(x, 0, 20);
    }
    for (int y = 0; y < height - 1; y++) {
      glVertex3d(width, y, 20);
      glVertex3d(width, y+0.05, 20);
      glVertex3d(0, y+0.05, 20);
      glVertex3d(0, y, 20);
    }

#if 0
    // full walls at 0,0
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, MAGENTA);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 1, 0);
    glVertex3d(0, 1, 1);
    glVertex3d(0, 0, 1);
 
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, GREY);
    glVertex3d(0, 0, 0);
    glVertex3d(1, 0, 0);
    glVertex3d(1, 0, 1);
    glVertex3d(0, 0, 1);
 #endif
    
    glEnd();
    glEndList();
  }
  void draw() {
    glCallList(displayListId);
  }
};

// Global variables: a camera, a checkerboard and some arrows.
Checkerboard checkerboard(CHECKB_WIDTH, CHECKB_HEIGHT);
Camera camera;
Arrows arrows[] = {
  Arrows(4, 4, 2),
  Arrows(6, 4, 2),
  Arrows(8, 4, 2),
};


// Application-specific initialization: Set up global lighting parameters
// and create display lists.
void init() {
  glEnable(GL_DEPTH_TEST);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, WHITE);
  glLightfv(GL_LIGHT0, GL_SPECULAR, WHITE);
  glMaterialfv(GL_FRONT, GL_SPECULAR, WHITE);
  glMaterialf(GL_FRONT, GL_SHININESS, 30);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  checkerboard.create();
}

// Draws one frame, the checkerboard then the arrows, from the current camera
// position.
void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(camera.getX(), camera.getY(), camera.getZ(),
            checkerboard.centerx(), checkerboard.centery(), checkerboard.centerz(),
            0.0, 1.0, 0.0);
  checkerboard.draw();
  for (int i = 0; i < sizeof arrows / sizeof(Arrows); i++) {
    arrows[i].update();
  }
  glFlush();
  glutSwapBuffers();
}

// On reshape, constructs a camera that perfectly fits the window.
void reshape(GLint w, GLint h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(40.0, GLfloat(w) / GLfloat(h), 1.0, 150.0);
  glMatrixMode(GL_MODELVIEW);
}

// Moves the camera according to the key pressed, then ask to refresh the
// display.
void special(int key, int, int) {
  switch (key) {
  case GLUT_KEY_LEFT: camera.moveLeft(); break;
  case GLUT_KEY_RIGHT: camera.moveRight(); break;
  case GLUT_KEY_UP: camera.moveUp(); break;
  case GLUT_KEY_DOWN: camera.moveDown(); break;
  }
  glutPostRedisplay();
}
void keyboard(unsigned char key,int x, int y) {
  switch (key) {
  case 'q':
  case 'Q':
  case 27:  // ESC
    exit(0);
  }  
}
void MouseFunc(int button, int state, int x, int y){
    if (button == 3) {
	  camera.zoomIn();
    } else if (button == 4) {
	  camera.zoomOut();
    }
}
void MotionFunc(int x, int y) {
  static int xx=-1;
  static int yy=-1;
  double dx, dy;
  
  dx = x - xx;
  dy = y - yy;

  if (xx != -1 && yy != -1 && fabs(dx) < 50 && fabs(dy) < 50) {
	camera.moveDxy(dx/10, dy/10);
  }
  xx = x; yy = y;
}


///////////////////////////////////////////////////////////////////////////

#define READ_BUFFER_SIZE (1<<15)
#define SKIP_BLANK(p) { while (*p != 0 && isspace(*p)) p++; }

static void wikiEulerAnglesToQuaternion(double yaw, double pitch, double roll, double *q) {
    double cy, sy, cp, sp, cr, sr;

    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    // [MV] Use pitch with inversed sign to get back to original quaternion
    cp = cos(-pitch * 0.5);
    sp = sin(-pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
}

int checkInput() {
  static char		bbb[READ_BUFFER_SIZE];
  static int		bbbi = 0;

  int 		i, n, t;
  int64_t		tstamp;
  char		*p, *q, *eq;
  int 		r, d, res;
  fd_set 		inset;
  fd_set 		outset;
  fd_set 		errset;
  struct timeval 	tout;
  static int inputPipeFd = STDIN_FILENO;
  res = 0;
  double 		a[16];
  double 		b[16];

  // TODO: Maybe do this more efficiently preparing and holding static empty fdsets
  FD_ZERO(&inset);
  FD_SET(inputPipeFd, &inset);
  FD_ZERO(&outset);
  FD_ZERO(&errset);
  FD_SET(inputPipeFd, &errset);
  tout.tv_sec = 0;
  tout.tv_usec = 1000;		// wait 1ms max
  // printf("["); fflush(stdout);
  r = select(inputPipeFd + 1, &inset, &outset, &errset, &tout);
  // printf("]"); fflush(stdout);

  if (r < 0) {
	// There we should probably check if timeout occured during multiple interruptions ...
	// Anyway for now we do nothing if select was just interrupted
	if (errno == EINTR) return(0);
	// this probably means some more serious error, do emergency landing
	printf("Error: %s:%d: select returned %d!\n", __FILE__, __LINE__, r);
	return(0);
  }
  // check for timeout expired
  if (r == 0) return(0);
  // if there is some problem with stdin, do emergency landing
  if (FD_ISSET(inputPipeFd, &errset)) return(0);
  if (! FD_ISSET(inputPipeFd, &inset)) return(0);
  if (bbbi >= READ_BUFFER_SIZE - 2) {
	printf("Error: %s:%d: read buffer full before read\n", __FILE__, __LINE__);	
	return(0);
  }
  // o.k. we have something on stdin
  n = read(inputPipeFd, bbb+bbbi, READ_BUFFER_SIZE - bbbi - 2);
  if (n == 0) return(0);
  if (n < 0) {
    printf("Error: %s:%d: problem on read!\n", __FILE__, __LINE__);
    return(0);
  }
  bbbi += n;
  if (bbbi >= READ_BUFFER_SIZE) {
	printf("Error: %s:%d: read buffer overflowed!\n", __FILE__, __LINE__);
	bbbi = 0;
	return(0);
  }
  bbb[bbbi] = 0;
  for (;;) {
	p = strchr(bbb, '\n');
	// no more newlines read we are done with parsing
	if (p == NULL) return(res);
	*p = 0;
	// o.k. we have whole line, parse it
	q = bbb;
#if 1
	{ static int counter = 0;
	  if (counter ++ % 127 == 0) {
	    printf("%s:%d: Parsing '%s'\n", __FILE__, __LINE__, bbb);
	    fflush(stdout);
	  }
	}
#endif
	SKIP_BLANK(q);
	if (strncmp(q, "quat", 4) == 0) {
	  q += 4;
	  for(i=0; i<4; i++) {
		a[i] = strtod(q, &eq);
		if (eq == q) goto skip;
		q = eq;
	  }
	  i = strtol(q, &eq, 10);
	  if (eq == q) i = 0;
	  arrows[i].setQuat(a);
	  res |= 1;
	} else if (strncmp(q, "rpy", 3) == 0) {
	  q += 3;
	  for(i=0; i<3; i++) {
		a[i] = strtod(q, &eq);
		if (eq == q) goto skip;
		q = eq;
	  }
	  wikiEulerAnglesToQuaternion(a[2], a[1], a[0], b);
	  i = strtol(q, &eq, 10);
	  if (eq == q) i = 0;
	  arrows[i].setQuat(b);
	  res |= 1;
	} else if (strncmp(q, "pose", 4) == 0) {
	  q += 4;
	  for(i=0; i<3; i++) {
		a[i] = strtod(q, &eq);
		if (eq == q) goto skip;
		q = eq;
	  }
	  i = strtol(q, &eq, 10);
	  if (eq == q || i<0 || i>=DIM(arrows) ) i = 0;
	  arrows[i].setPose(a[0]*100, a[1]*100, a[2]*100);
	} else {
	  printf("%s:%d: Ignoring '%s'\n", __FILE__, __LINE__, bbb);
	  fflush(stdout);
	}
  skip:
	d = bbb+bbbi-p-1;
	memmove(bbb, p+1, d);
	bbbi = d;
  }
  return(0);
}


// Requests to draw the next frame.
void timer(int v) {
  checkInput();
  glutPostRedisplay();
  glutTimerFunc(1000/30, timer, v);
}


// Initializes GLUT and enters the main loop.
int main(int argc, char** argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowPosition(80, 80);
  glutInitWindowSize(800, 600);
  glutCreateWindow("Quaternion Arrows");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutTimerFunc(100, timer, 0);
  init();
  glutMainLoop();
}
