/*
 * @file CountingBees.h
 */

 
#ifndef COUNTING_BEES_H
#define COUNTING_BEES_H


// ------------ Include Files -------------------------------------------------
#include <vector>


// ------------ defines -------------------------------------------------------
// The direction of bee box gate, it use the reverse clock direction.
typedef enum tagBEEBOX_GATE_FACING
{
    BEEBOX_GATE_UNKNOWN  = 0,
    BEEBOX_GATE_0        = 1,     // 0   degree, reverse clock direction(from left to right)
    BEEBOX_GATE_90       = 2,     // 90  degree (from bottom to top)
    BEEBOX_GATE_180      = 3,     // 180 degree(from right to left)
    BEEBOX_GATE_270      = 4,     // 270 degree(from top to bottom)
} BEEBOX_GATE_FACING;
 

//
#define TRACE_NODE_FLAG_COUNTED  0x0001


// ------------ class defines -------------------------------------------------
class CountingBees
{
protected:
    typedef struct tagBEEBOX_GATE
    {
        int                xLeft;
        int                yTop;
        int                xRight;
        int                yBottom;
        BEEBOX_GATE_FACING nDirection;
    } BEEBOX_GATE, *PBEEBOX_GATE;

    typedef struct tagTRACK_NODE
    {
        int   nFrame;
        int   nID;
        float fPosX;
        float fPosY;
        float fWidth;
        float fHeight;
        int   nFlags;
    } TRACK_NODE, *PTRACK_NODE;

    typedef struct tagTRACK_POINTS
    {
        int   nID;
        float fPosX;
        float fPosY;
    } TRACK_POINT, *PTRACK_POIN;

    typedef std::vector<TRACK_NODE>    vectorNodes;
    typedef std::vector<TRACK_POINT>   vectorPoints;
    typedef std::vector<vectorPoints>  vectorTrackList;

public:
    CountingBees(int nResetDuration, int xLeft, int yTop, int xRight, int xBottom, BEEBOX_GATE_FACING nDirection);

public:
    bool Count(int& cnInBees, int& cnOutBees);
        
public:
    void Update(int& beesInTotal, int& beesOutTotal, int& nFrame, int idTrack, float x, float y, float width, float height);

protected:
    float GetTrackAngle(const TRACK_POINT& ptStart, const TRACK_POINT& ptEnd);
    void  SortVectorInts(const std::vector<int>& rgIntSrc, std::vector<int>& rgIntDest);
    bool  GetMaxMinFrame(int& nFrameMin, int& nFrameMax);
    
protected:
    BEEBOX_GATE             m_gate;
    int                     m_nStartTime;
    int                     m_nResetDuration;
    std::vector<TRACK_NODE> m_rgTrackNodes;
};


#endif // COUNTING_BEES_H

