/**
 * @file <argos3/plugins/robots/e-puck/simulator/qtopengl_epuck.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "qtopengl_epuck.h"
#include "epuck_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real WHEEL_DIAMETER              = 0.041f;
   static const Real WHEEL_RADIUS                = WHEEL_DIAMETER * 0.5f;
   static const Real WHEEL_WIDTH                 = 0.01f;
   static const Real HALF_WHEEL_WIDTH            = WHEEL_WIDTH * 0.5f;
   static const Real INTERWHEEL_DISTANCE         = 0.053f;
   static const Real HALF_INTERWHEEL_DISTANCE    = INTERWHEEL_DISTANCE * 0.5f;

   static const Real CHASSIS_ELEVATION           = 0.005f;                            // to be checked!
   static const Real HALF_CHASSIS_LENGTH         = 0.0275f;                             // to be checked!
   static const Real HALF_CHASSIS_WIDTH          = HALF_INTERWHEEL_DISTANCE - HALF_WHEEL_WIDTH;

   static const Real BODY_RADIUS                 = 0.035f;
   static const Real BODY_ELEVATION              = WHEEL_DIAMETER + CHASSIS_ELEVATION; // to be checked!
   static const Real BODY_HEIGHT                 = 0.03f;                              // to be checked!

   static const Real LED_ELEVATION               = BODY_ELEVATION;
   static const Real LED_HEIGHT                  = 0.01;                               // to be checked!
   static const Real LED_UPPER_RING_INNER_RADIUS = 0.8 * BODY_RADIUS;
   static const Real LED_UPPER_RING_OUTHER_RADIUS = 1.1 * BODY_RADIUS;

   static const Real RGB_LED_ELEVATION               = BODY_ELEVATION+BODY_HEIGHT; //to be checked
   static const Real RGB_LED_HEIGHT                  = 0.002;                               // to be checked!
   static const Real RGB_LED_UPPER_RING_INNER_RADIUS = 0.8 * BODY_RADIUS;
   static const Real RGB_LED_UPPER_RING_OUTER_RADIUS = 1 * BODY_RADIUS; // to be checked

   /* Camera */
   static const Real CAMERA_ELEVATION            = BODY_HEIGHT+BODY_ELEVATION ;
   static const Real CAMERA_RADIUS               = 0.007;
   static const Real CAMERA_HEIGHT               = 0.04f;

   static const UInt8 EPUCK_COMPONENTS_NUMBER        = 6;

   /****************************************/
   /****************************************/

   CQTOpenGLEPuck::CQTOpenGLEPuck() :
      m_unVertices(40),
      m_fLEDAngleSlice(360.0f / 8.0f),
      m_fRGBLEDAngleSlice(360.0f / 3.0f) {
      LOG << m_fRGBLEDAngleSlice << std::endl;
      LOG << m_fLEDAngleSlice << std::endl;
      /* Reserve the needed display lists */
      m_unLists = glGenLists(EPUCK_COMPONENTS_NUMBER);

      /* Assign indices for better referencing (later) */
      m_unWheelList   = m_unLists;
      m_unChassisList = m_unLists + 1;
      m_unBodyList    = m_unLists + 2;
      m_unLEDList     = m_unLists + 3;
      m_unRGBLEDList  = m_unLists + 4;
      m_unCameraList  = m_unLists + 5;
      /* Create the wheel display list */
      glNewList(m_unWheelList, GL_COMPILE);
      RenderWheel();
      glEndList();

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the chassis display list */
      glNewList(m_unChassisList, GL_COMPILE);
      RenderChassis();
      glEndList();

      /* Create the LED display list */
      glNewList(m_unLEDList, GL_COMPILE);
      RenderLED();
      glEndList();

      /* Create the RGB LED display list */
      glNewList(m_unRGBLEDList, GL_COMPILE);
      RenderRGBLED();
      glEndList();

      /* Create the Camera display list */
      glNewList(m_unCameraList, GL_COMPILE);
      RenderCamera();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLEPuck::~CQTOpenGLEPuck() {
      glDeleteLists(m_unLists, EPUCK_COMPONENTS_NUMBER);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::Draw(CEPuckEntity& c_entity) {
      /* Place the chassis */
      glCallList(m_unChassisList);
      /* Place the body */
      glCallList(m_unBodyList);
      /* Place the wheels */
      glPushMatrix();
      glTranslatef(0.0f, HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      glPushMatrix();
      glTranslatef(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      /* Place the LEDs */
      glPushMatrix();
      CLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      for(UInt8 i = 0; i < 11; i++) {
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         glRotatef(-m_fLEDAngleSlice, 0.0f, 0.0f, 1.0f);
         SetLEDMaterial(cColor.GetRed(),
                        cColor.GetGreen(),
                        cColor.GetBlue());
         glCallList(m_unLEDList);
      }
      glPopMatrix();
      /* Place the RGB LEDs */
      glPushMatrix();
      glRotatef(m_fRGBLEDAngleSlice, 0.0f, 0.0f, 1.0f);
      for(UInt8 i = 8; i < 11; i++) {
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         glRotatef(-m_fRGBLEDAngleSlice, 0.0f, 0.0f, 1.0f);
         SetLEDMaterial(cColor.GetRed(),
                        cColor.GetGreen(),
                        cColor.GetBlue());
         glCallList(m_unRGBLEDList);
      }
      glPopMatrix();
      /* Place the camera */
      glCallList(m_unCameraList);

   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::SetGreenPlasticMaterial() {
      const GLfloat pfColor[]     = {   0.0f, 1.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }
   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::SetWhitePlasticMaterial() {
      const GLfloat pfColor[]     = {   1.0f, 1.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }
   void CQTOpenGLEPuck::SetGrayPlasticMaterial() {
      const GLfloat pfColor[]     = {   0.5f, 0.5f, 0.5f, 0.6f };
      const GLfloat pfSpecular[]  = {   0.1f, 0.1f, 0.1f, 1.0f };
      const GLfloat pfShininess[] = { 20.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }
   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::SetRedPlasticMaterial() {
      const GLfloat pfColor[]     = {   1.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::SetCircuitBoardMaterial() {
      const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
      const GLfloat pfShininess[] = { 10.0f                  };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::SetLEDMaterial(GLfloat f_red,
                                       GLfloat f_green,
                                       GLfloat f_blue) {
      const GLfloat fEmissionFactor = 10.0f;
      const GLfloat pfColor[]     = {f_red, f_green, f_blue, 1.0f };
      const GLfloat pfSpecular[]  = {0.0f,  0.0f, 0.0f, 1.0f };
      const GLfloat pfShininess[] = {0.0f};
      const GLfloat pfEmission[]  = {f_red * fEmissionFactor,
                                     f_green * fEmissionFactor,
                                     f_blue * fEmissionFactor,
                                     1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::RenderWheel() {
      /* Set material */
      SetGrayPlasticMaterial();
      /* Right side */
      CVector2 cVertex(WHEEL_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector3 cNormal(-1.0f, -1.0f, 0.0f);
      cNormal.Normalize();
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Left side */
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cNormal.Set(-1.0f, 1.0f, 0.0f);
      cNormal.Normalize();
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Tire */
      cNormal.Set(1.0f, 0.0f, 0.0f);
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         glVertex3f(cVertex.GetX(),  HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::RenderChassis() {
      /* Set material */
      SetGreenPlasticMaterial();
      /* This part covers the bottom face (parallel to XY) */
      glBegin(GL_QUADS);
      /* Bottom face */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUAD_STRIP);
      /* Starting side */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* South face */
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* East face */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* North face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* West face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::RenderBody() {
      /* Set material */
      SetGreenPlasticMaterial();
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(-CRadians::TWO_PI / m_unVertices);
      /* Bottom part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Side surface */
      cAngle = -cAngle;
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top part */
      glBegin(GL_POLYGON);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Triangle to set the direction */
      SetLEDMaterial(1.0f, 1.0f, 0.0f);
      glBegin(GL_TRIANGLES);
      glVertex3f( BODY_RADIUS * 0.7,               0.0f, BODY_ELEVATION + BODY_HEIGHT +  0.001f);
      glVertex3f(-BODY_RADIUS * 0.7,  BODY_RADIUS * 0.4, BODY_ELEVATION + BODY_HEIGHT +  0.001f);
      glVertex3f(-BODY_RADIUS * 0.7, -BODY_RADIUS * 0.4, BODY_ELEVATION + BODY_HEIGHT +  0.001f);
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck::RenderLED() {
      /* Side surface */
      CVector2 cVertex(LED_UPPER_RING_OUTHER_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector2 cNormal(1.0f, 0.0f);
      /* LEDs */
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices / 8; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
//      /* RGB LEDs */
//      glBegin(GL_QUAD_STRIP);
//      for(GLuint i = 0; i <= m_unVertices / 3; i++) {
//         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
//         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
//         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION);
//         cVertex.Rotate(cAngle);
//         cNormal.Rotate(cAngle);
//      }
//      glEnd();

      /* Top surface  */
      cVertex.Set(LED_UPPER_RING_OUTHER_RADIUS, 0.0f);
      CVector2 cVertex2(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      /* LEDs */
      glBegin(GL_QUAD_STRIP);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices / 8; i++) {
         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), LED_ELEVATION + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
         cVertex.Rotate(cAngle);
         cVertex2.Rotate(cAngle);
      }
      glEnd();
//      /* RGB LEDs */
//      glBegin(GL_QUAD_STRIP);
//      glNormal3f(0.0f, 0.0f, 1.0f);
//      for(GLuint i = 0; i <= m_unVertices / 3; i++) {
//         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
//         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
//         cVertex.Rotate(cAngle);
//         cVertex2.Rotate(cAngle);
//      }
//      glEnd();
   }

   void CQTOpenGLEPuck::RenderRGBLED() {
      /* Side surface */
      CVector2 cVertex(RGB_LED_UPPER_RING_OUTER_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector2 cNormal(1.0f, 0.0f);
      /* RGB LEDs */
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices / 3; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();

      /* Top surface  */
      cVertex.Set(RGB_LED_UPPER_RING_OUTER_RADIUS, 0.0f);
      CVector2 cVertex2(RGB_LED_UPPER_RING_INNER_RADIUS, 0.0f);

      /* RGB LEDs */
      glBegin(GL_QUAD_STRIP);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices / 3; i++) {
         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), RGB_LED_ELEVATION + RGB_LED_HEIGHT);
         cVertex.Rotate(cAngle);
         cVertex2.Rotate(cAngle);
      }
      glEnd();
   }



   void CQTOpenGLEPuck::RenderCamera() {
      /* Set material */
      //SetWhitePlasticMaterial();
      SetWhitePlasticMaterial();
      CVector2 cVertex(CAMERA_RADIUS, 0.0f);
      CRadians cAngle(-CRadians::TWO_PI / m_unVertices);
      /* Bottom part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), CAMERA_ELEVATION);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Side surface */
      cAngle = -cAngle;
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(CAMERA_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), CAMERA_ELEVATION + CAMERA_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), CAMERA_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, 1.0f);
      cVertex.Set(CAMERA_RADIUS, 0.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), CAMERA_ELEVATION + CAMERA_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();
   }
   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawEPuckNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CEPuckEntity& c_entity) {
         static CQTOpenGLEPuck m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawEPuckSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CEPuckEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawEPuckNormal, CEPuckEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawEPuckSelected, CEPuckEntity);

   /****************************************/
   /****************************************/

}
