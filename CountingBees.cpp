/*
 * @file CountingBees.cpp
 */


// ------------ Include Files -------------------------------------------------
#include "vector"
#include "time.h"
#include "math.h"
#include "string.h"
#include "CountingBees.h"


// ------------ defines -------------------------------------------------------
using namespace std;

#define PI  3.1415926535


// -------- Constructor and destructor ----------------------------------------
CountingBees::CountingBees(
    int                nResetDuration,
    int                xLeft,
    int                yTop,
    int                xRight,
    int                yBottom,
    BEEBOX_GATE_FACING nDirection) :
    m_nStartTime(-1),
    m_nResetDuration(nResetDuration),
    m_gate{xLeft, yTop, xRight, yBottom, nDirection}
{
    m_gate.xLeft = xLeft;
    m_gate.xRight = xRight;
    m_gate.yTop = yTop;
    m_gate.yBottom = yBottom;
    m_gate.nDirection = nDirection;
}


//*****************************************************************************
//   @name CountingBees::Update
//   @desc
//*****************************************************************************
void CountingBees::Update(
int& beesInTotal, 
int& beesOutTotal,
    int&   nFrame, 
    int   nIDTrack, 
    float x, 
    float y, 
    float width, 
    float height)
{
    TRACK_NODE node;
    time_t     timeCurrent;

    time(&timeCurrent);
    if (m_nStartTime < 0)
    {
        m_nStartTime = static_cast<int>(timeCurrent);
    }
    else if (timeCurrent - m_nStartTime >= m_nResetDuration)
    {
        nFrame = 0;
	beesInTotal = 0;
	beesOutTotal = 0;
        m_rgTrackNodes.clear();
        m_nStartTime = static_cast<int>(timeCurrent);
    }

    node.nFrame  = nFrame;
    node.nID     = nIDTrack;
    node.fPosX   = x;
    node.fPosY   = y;
    node.fWidth  = width;
    node.fHeight = height;

    m_rgTrackNodes.push_back(node);
}


//*****************************************************************************
//   @name CountingBees::Count
//   @desc
//*****************************************************************************
bool CountingBees::Count(
    int& cnBeesIn, 
    int& cnBeesOut)
{
    vectorTrackList rgPointsById;
    vectorPoints    rgPoints;
    vector<int>     rgCurrIndice, rgBackIndice, rgCurrSecondIds, rgBackSecondIds, rgCurrSecondIds2, rgBackSecondIds2;
    TRACK_POINT     point, ptStart, ptEnd;
    size_t          i, j, k;
    float           fAngle;
    bool            bRet = true;
    int             nFrame, nFrameMin, nFrameMax, nFrameTrack, nFreq, nID, nIDCurr, cnSeconds, cnCurrInBees, cnCurrOutBees;

    nFreq     = 30;  // 统计频率
    i         = 29;  // 初始获取值设定，即最开始的一个时间间隔的大小
    cnSeconds = 0;

    GetMaxMinFrame(nFrameMin, nFrameMax);

    cnBeesIn = cnBeesOut = 0;

    for (nFrame = nFrameMin; nFrame <= nFrameMax; nFrame += nFreq)
    {
        cnSeconds++;

        //分别获取当前秒包含帧的记录，和当前秒后100帧的记录
        rgCurrIndice.clear();
        rgBackIndice.clear();
        for (i = 0; i < m_rgTrackNodes.size(); i++)
        {

            nFrameTrack = m_rgTrackNodes[i].nFrame;
            if (nFrame - nFreq <= nFrameTrack && nFrameTrack <= nFrame)
            {
                //当前秒之前所有帧的记录
                rgCurrIndice.push_back(static_cast<int>(i));
            }
            else if (nFrame < nFrameTrack && nFrameTrack <= nFrame + 100)
            {
                //当前秒以后的所有帧的记录                    
                rgBackIndice.push_back(static_cast<int>(i));
            }
        }

        //获取当前秒以前帧的跟踪轨迹ID
        rgCurrSecondIds.clear();
        for (i = 0; i < rgCurrIndice.size(); i++)
        {
            nID = m_rgTrackNodes[rgCurrIndice[i]].nID;

            // Add to the id list, only if there is no duplicated one
            for (j = 0; j < rgCurrSecondIds.size(); j++)
            {
                if (nID == rgCurrSecondIds[j])
                {
                    break;
                }
            }
            if (j >= rgCurrSecondIds.size())
            {
                rgCurrSecondIds.push_back(nID);
            }
        }
        SortVectorInts(rgCurrSecondIds, rgCurrSecondIds2);

        //获取当前秒以后帧的跟踪轨迹ID
        rgBackSecondIds.clear();
        for (i = 0; i < rgBackIndice.size(); i++)
        {
            nID = m_rgTrackNodes[rgBackIndice[i]].nID;

            // Add to the id list, only if there is no duplicated one
            for (j = 0; j < rgBackSecondIds.size(); j++)
            {
                if (nID == rgBackSecondIds[j])
                {
                    break;
                }
            }
            if (j >= rgBackSecondIds.size())
            {
                rgBackSecondIds.push_back(nID);
            }
        }
        SortVectorInts(rgBackSecondIds, rgBackSecondIds2);

        //按照所有ID进行纪录
        rgPointsById.clear();
        for (i = 0; i < rgCurrSecondIds2.size(); i++)
        {
            nIDCurr = rgCurrSecondIds2[i];

            rgPoints.clear();
            for (j = 0; j < rgCurrIndice.size(); j++)
            {
                TRACK_NODE& node = m_rgTrackNodes[rgCurrIndice[j]];
                nID = node.nID;
                if (nID == nIDCurr)
                {
                    // Check if the id is in rgBackSecondIds2
                    for (k = 0; k < rgBackSecondIds2.size(); k++)
                    {
                        if (nID == rgBackSecondIds2[k])
                        {
                            break;
                        }
                    }
                    if (k >= rgBackSecondIds2.size())
                    {
                        // The id isn't in rgBackSecondIds2
                        point.nID   = nID;

                        // Set the position of the track to the center of the object's bounding box
                        point.fPosX = node.fPosX + 0.5f * node.fWidth;
                        point.fPosY = node.fPosY + 0.5f * node.fHeight;
                        rgPoints.push_back(point);
                    }
                }
            }

            if (rgPoints.size() > 0)
            {
                rgPointsById.push_back(rgPoints);
            }
        }

        //提取有效记录中的第一点和最后一点,连线与水平线组成角度,对进出情况进行判断
        cnCurrInBees  = 0;
        cnCurrOutBees = 0;
        for (i = 0; i < rgPointsById.size(); i++)
        {
            vectorPoints& rgPts = rgPointsById[i];
            //if (rgPts.size() < 2)
            //{
            //    continue;
            //}
            ptStart = rgPts[0];
            ptEnd = rgPts[rgPts.size() - 1];
            fAngle = GetTrackAngle(ptStart, ptEnd);
            if (   m_gate.xLeft <= ptEnd.fPosX && ptEnd.fPosX <= m_gate.xRight
                && m_gate.yTop  <= ptEnd.fPosY && ptEnd.fPosY <= m_gate.yBottom)
            {
                switch (m_gate.nDirection)
                {
                case BEEBOX_GATE_270:
                    if (fAngle <= 180)
                    {
                        cnCurrInBees++;
                    }
                    else
                    {
                        cnCurrOutBees++;
                    }
                    break;

                case BEEBOX_GATE_180:
                    if (90 < fAngle && fAngle < 270)
                    {
                        cnCurrOutBees++;
                    }
                    else
                    {
                        cnCurrInBees++;
                    }                    
                    break;

                case BEEBOX_GATE_0:
                    if (90 < fAngle && fAngle < 270)
                    {
                        cnCurrInBees++;
                    }
                    else
                    {
                        cnCurrOutBees++;
                    }
                    break;

                case BEEBOX_GATE_90:
                    if (fAngle > 180)
                    {
                        cnCurrInBees++;
                    }
                    else
                    {
                        cnCurrOutBees++;
                    }                    
                    break;

                default:
                    break;
                }
            }
        }

        cnBeesIn  += cnCurrInBees;
        cnBeesOut += cnCurrOutBees;
    }

    return bRet;
}


//*****************************************************************************
//   @name  CountingBees::GetMaxMinFrame
//   @desc 
//*****************************************************************************
bool CountingBees::GetMaxMinFrame(
    int& nFrameMin, 
    int& nFrameMax)
{
    size_t i;
    bool   bRet = true;
    int    nFrame;

    if (m_rgTrackNodes.size() > 0)
    {
        nFrameMin = nFrameMax = m_rgTrackNodes[0].nFrame;
        for (i = 0; i < m_rgTrackNodes.size(); i++)
        {
            nFrame = m_rgTrackNodes[i].nFrame;
            nFrameMin = nFrameMin > nFrame ? nFrame : nFrameMin;
            nFrameMax = nFrameMax < nFrame ? nFrame : nFrameMax;
        }
    }
    else
    {
        nFrameMin = nFrameMax = -1;
    }

    return bRet;
}


//*****************************************************************************
//   @name  CountingBees::GetTrackAngle
//   @desc 
//*****************************************************************************
float CountingBees::GetTrackAngle(
    const TRACK_POINT& ptStart,
    const TRACK_POINT& ptEnd)
{
    double dAngle;
    double k = atan2(ptStart.fPosY - ptEnd.fPosY, ptEnd.fPosX - ptStart.fPosX);
    if (ptEnd.fPosY <= ptStart.fPosY)
    {
        dAngle = (k * 180 / PI);
    }
    else
    {
        dAngle = (360 + k * 180 / PI);
    }

    return static_cast<float>(dAngle);
}


//*****************************************************************************
//   @name  CountingBees::SortVectorInts
//   @desc 
//*****************************************************************************
void CountingBees::SortVectorInts(
    const vector<int>& rgIntSrc,
    vector<int>&       rgIntDest)
{
    size_t i, j;

    rgIntDest.clear();
    for (i = 0; i < rgIntSrc.size(); i++)
    {
        rgIntDest.push_back(rgIntSrc[i]);
    }

    for (i = 0; i + 1 < rgIntDest.size(); i++)
    {
        for (j = i + 1; j < rgIntDest.size(); j++)
        {
            if (rgIntDest[i] > rgIntDest[j])
            {
                int temp     = rgIntDest[i];
                rgIntDest[i] = rgIntDest[j];
                rgIntDest[j] = temp;
            }
        }
    }
}


