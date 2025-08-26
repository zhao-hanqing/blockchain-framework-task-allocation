/**
 * @file <argos3/plugins/robots/e-puck/simulator/epuck_omnidirectional_camera_sensor.cpp>
 *
 * @author Gianpiero Francesca - <gianpiero.francesca@ulb.ac.be>
 * @author Lorenzo Garattoni - <lgaratto@ulb.ac.be>
 */
#include "epuck_omnidirectional_camera_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/entities/omnidirectional_camera_equipped_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>

namespace argos {

/****************************************/
/****************************************/

class CLEDCheckOperation : public CPositionalIndex<CLEDEntity>::COperation {

public:

    CLEDCheckOperation(CCI_EPuckOmnidirectionalCameraSensor::TBlobList& t_blobs,
                       COmnidirectionalCameraEquippedEntity& c_omnicam_entity,
                       CEmbodiedEntity& c_embodied_entity,
                       CControllableEntity& c_controllable_entity,
                       bool b_show_rays,Real f_distance_std_dev,CRandom::CRNG* pcRNG) :
        m_tBlobs(t_blobs),
        m_cOmnicamEntity(c_omnicam_entity),
        m_cEmbodiedEntity(c_embodied_entity),
        m_cControllableEntity(c_controllable_entity),
        m_bShowRays(b_show_rays),
        m_fDistanceNoiseStdDev(f_distance_std_dev),
        m_pcRNG(pcRNG){
        m_pcRootSensingEntity = &m_cEmbodiedEntity.GetParent();

    }
    virtual ~CLEDCheckOperation() {
        while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
        }
    }

    virtual bool operator()(CLEDEntity& c_led) {
        /* Process this LED only if it's lit */
        if(c_led.GetColor() != CColor::BLACK) {
            if(c_led.HasParent()) {
                /* Filter out the LEDs belonging to the sensing entity by checking if they share the same parent entity */
                m_pcRootOfLEDEntity = &c_led.GetParent();
                while(m_pcRootOfLEDEntity->HasParent()) m_pcRootOfLEDEntity = &m_pcRootOfLEDEntity->GetParent();
                if(m_pcRootSensingEntity == m_pcRootOfLEDEntity) {
                    return true;
                }
            }
            /* If we are here, it's because the LED must be processed */
            m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());
            m_cLEDRelativePos = c_led.GetPosition();
            m_cLEDRelativePos -= m_cCameraPos;
            m_cLEDRelativePosXY.Set(m_cLEDRelativePos.GetX(),
                                    m_cLEDRelativePos.GetY());
            if(m_cLEDRelativePos.GetZ() < m_cCameraPos.GetZ() &&
                    m_cLEDRelativePos.Length() < m_fGroundHalfRange &&
                                !GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem,
                                                                          m_cOcclusionCheckRay,
                                                                          m_cEmbodiedEntity)) {
                if(m_fDistanceNoiseStdDev > 0.0f) {
                    m_cLEDRelativePosXY += CVector2(
                                m_pcRNG->Gaussian(m_fDistanceNoiseStdDev),
                                m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE));
                }
                Real fDistance=m_cLEDRelativePosXY.Length() * 100.0f;
                /*
                 * NOTE the area is calculated as 95.644*exp(-0.11f*fDistance). Such equation
                 * has been calculated from empirical data.
                   */
                m_tBlobs.push_back(new CCI_EPuckOmnidirectionalCameraSensor::SBlob(
                                       c_led.GetColor(),
                                       NormalizedDifference(m_cLEDRelativePosXY.Angle(), m_cCameraOrient),
                                       fDistance,95.644*exp(-0.11f*fDistance)));
                if(m_bShowRays) {
                    m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPos, c_led.GetPosition()));
                }
            }
        }
        return true;
    }

    void Setup(Real f_ground_half_range) {
        while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
        }
        m_fGroundHalfRange = f_ground_half_range;
        m_cEmbodiedEntity.GetOriginAnchor().Orientation.ToEulerAngles(m_cCameraOrient, m_cTmp1, m_cTmp2);
        m_cCameraPos = m_cEmbodiedEntity.GetOriginAnchor().Position;
        m_cCameraPos += m_cOmnicamEntity.GetOffset();
        m_cOcclusionCheckRay.SetStart(m_cCameraPos);
    }

private:

    CCI_EPuckOmnidirectionalCameraSensor::TBlobList& m_tBlobs;
    COmnidirectionalCameraEquippedEntity& m_cOmnicamEntity;
    CEmbodiedEntity& m_cEmbodiedEntity;
    CControllableEntity& m_cControllableEntity;
    Real m_fGroundHalfRange;
    bool m_bShowRays;
    CEntity* m_pcRootSensingEntity;
    CEntity* m_pcRootOfLEDEntity;
    CVector3 m_cCameraPos;
    CRadians m_cCameraOrient;
    CRadians m_cTmp1, m_cTmp2;
    CVector3 m_cLEDRelativePos;
    CVector2 m_cLEDRelativePosXY;
    SEmbodiedEntityIntersectionItem m_sIntersectionItem;
    CRay3 m_cOcclusionCheckRay;
    Real              m_fDistanceNoiseStdDev;
    CRandom::CRNG*    m_pcRNG;

};

/****************************************/
/****************************************/

CEPuckOmnidirectionalCameraSensor::CEPuckOmnidirectionalCameraSensor() :
    m_bEnabled(false),
    m_pcOmnicamEntity(NULL),
    m_pcControllableEntity(NULL),
    m_pcEmbodiedEntity(NULL),
    m_pcLEDIndex(NULL),
    m_pcEmbodiedIndex(NULL),
    m_fDistanceNoiseStdDev(0.0f),
    m_pcRNG(NULL),
    m_bShowRays(false) {}

/****************************************/
/****************************************/

CEPuckOmnidirectionalCameraSensor::~CEPuckOmnidirectionalCameraSensor() {
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::SetRobot(CComposableEntity& c_entity) {
    /* Get and enable omndirectional camera equipped entity */
    m_pcOmnicamEntity = &(c_entity.GetComponent<COmnidirectionalCameraEquippedEntity>("omnidirectional_camera"));
    /* Get controllable entity */
    m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
    /* Get embodied entity */
    m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Init(TConfigurationNode& t_tree) {
    try {
        /* Parent class init */
        CCI_EPuckOmnidirectionalCameraSensor::Init(t_tree);
        /* Show rays? */
        GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
        /* Parse noise */
        GetNodeAttributeOrDefault(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, m_fDistanceNoiseStdDev);
        if(m_fDistanceNoiseStdDev > 0.0f) {
            m_pcRNG = CRandom::CreateRNG("argos");
        }
        /* Get LED medium from id specified in the XML */
        std::string strMedium;
        GetNodeAttribute(t_tree, "medium", strMedium);
        m_pcLEDIndex = &(CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium).GetIndex());
        /* Create check operation */
        m_pcOperation = new CLEDCheckOperation(m_sReadings.BlobList,
                                               *m_pcOmnicamEntity,
                                               *m_pcEmbodiedEntity,
                                               *m_pcControllableEntity,
                                               m_bShowRays,m_fDistanceNoiseStdDev,m_pcRNG);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the colored blob omnidirectional camera rotzonly sensor", ex);
    }
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Update() {
    if(m_bEnabled) {
        /* Increase data counter */
        ++m_sReadings.Counter;
        /* Calculate range on the ground */
        CVector3 cCameraPos = m_pcOmnicamEntity->GetOffset();
        cCameraPos += m_pcEmbodiedEntity->GetOriginAnchor().Position;
        Real fGroundHalfRange = cCameraPos.GetZ() * Tan(m_pcOmnicamEntity->GetAperture());
        /* Prepare the operation */
        m_pcOperation->Setup(fGroundHalfRange);
        /* Go through LED entities in sphere range */
        m_pcLEDIndex->ForEntitiesInSphereRange(
                    CVector3(cCameraPos.GetX(),
                             cCameraPos.GetY(),
                             cCameraPos.GetZ() * 0.5f),
                    fGroundHalfRange,
                    *m_pcOperation);
    }
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Reset() {
    m_sReadings.Counter = 0;
    m_sReadings.BlobList.clear();
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Destroy() {
    delete m_pcOperation;
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Enable() {
    m_pcOmnicamEntity->Enable();
    m_bEnabled = true;
}

/****************************************/
/****************************************/

void CEPuckOmnidirectionalCameraSensor::Disable() {
    m_pcOmnicamEntity->Disable();
    m_bEnabled = false;
}

/****************************************/
/****************************************/

REGISTER_SENSOR(CEPuckOmnidirectionalCameraSensor,
                "epuck_omnidirectional_camera", "rot_z_only",
                "Lorenzo Garattoni [lgaratto@ulb.ac.be]",
                "1.0",
                "A generic omnidirectional camera sensor to detect colored blobs.",
                "This sensor accesses an omnidirectional camera that detects colored blobs. The\n"
                "sensor returns a list of blobs, each defined by a color and a position with\n"
                "respect to the robot reference point on the ground. In controllers, you must\n"
                "include the ci_colored_blob_omnidirectional_camera_sensor.h header.\n\n"
                "REQUIRED XML CONFIGURATION\n\n"
                "  <controllers>\n"
                "    ...\n"
                "    <my_controller ...>\n"
                "      ...\n"
                "      <sensors>\n"
                "        ...\n"
                "        <colored_blob_omnidirectional_camera implementation=\"rot_z_only\"\n"
                "                                             medium=\"leds\" />\n"
                "        ...\n"
                "      </sensors>\n"
                "      ...\n"
                "    </my_controller>\n"
                "    ...\n"
                "  </controllers>\n\n"
                "The 'medium' attribute must be set to the id of the leds medium declared in the\n"
                "<media> section.\n\n"
                "OPTIONAL XML CONFIGURATION\n\n"
                "It is possible to draw the rays shot by the camera sensor in the OpenGL\n"
                "visualization. This can be useful for sensor debugging but also to understand\n"
                "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                "they are not obstructed and in purple when they are. In case a ray is\n"
                "obstructed, a black dot is drawn where the intersection occurred.\n"
                "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                "example:\n\n"
                "  <controllers>\n"
                "    ...\n"
                "    <my_controller ...>\n"
                "      ...\n"
                "      <sensors>\n"
                "        ...\n"
                "        <colored_blob_omnidirectional_camera implementation=\"rot_z_only\"\n"
                "                                             medium=\"leds\" />\n"
                "                                             show_rays=\"true\" />\n"
                "        ...\n"
                "      </sensors>\n"
                "      ...\n"
                "    </my_controller>\n"
                "    ...\n"
                "  </controllers>\n\n"
                "It is possible to add uniform noise to the blobs, thus matching the\n"
                "characteristics of a real robot better. This can be done with the attribute\n"
                "\"noise_std_dev\".\n\n"
                "  <controllers>\n"
                "    ...\n"
                "    <my_controller ...>\n"
                "      ...\n"
                "      <sensors>\n"
                "        ...\n"
                "        <colored_blob_omnidirectional_camera implementation=\"rot_z_only\"\n"
                "                                             medium=\"leds\" />\n"
                "                                             noise_std_dev=\"0.1\" />\n"
                "        ...\n"
                "      </sensors>\n"
                "      ...\n"
                "    </my_controller>\n"
                "    ...\n"
                "  </controllers>\n",
                "Usable"
                );

}
