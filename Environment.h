#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <QObject>
#include "Util.h"

typedef struct State
{
    int     stateWidth;
    int     stateHeight;
    int     stateCol;
    int     stateRow;
    bool    isStart;
    bool    isGoal;
    bool    isObstacle;
    bool    isPath;
    bool    isEmpty;
    bool    isRobot;
    int     xInit;
    int     yInit;
    int     idRobot;
}State;

class Environment : public QObject
{
    Q_OBJECT
public:
    explicit Environment(QObject *parent = 0);
    bool setObstacle ( int i, int j );
    bool clearObstacle ( int i, int j );
    bool setEmpty ( int i, int j );
    bool setGoal ( int i, int j );
    bool setStart ( int i, int j );
    bool setNewStatePath ( int i, int j );
    bool setRobot ( int i, int j, int id );

    void getStart ( int &i, int &j );
    void getGoal ( int &i, int &j );

    void clearPath ();
    void clearObstacles ();

    void restartEnvironment ();

    void updateStates ( int numberOfLines, int numberOfCols );
signals:
    
public slots:

public:
    State *start;
    State *goal;
    State ***states;
    int nRows;
    int nCols;
};

#endif // ENVIRONMENT_H
