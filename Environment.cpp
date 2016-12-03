#include "Environment.h"
#include <QDebug>

Environment::Environment(QObject *parent) :
    QObject(parent)
{
    start = NULL;
    goal = NULL;
    nRows = NUMBER_ROWS;
    nCols = NUMBER_COLS;
    states = new State **[ nRows ];
    for ( int i = 0; i < nRows; i++ )
    {
        states [ i ] = new State *[ nCols ];
        for ( int j = 0; j < nCols; j++ )
        {
            states [ i ][ j ] = new State;
            states [ i ][ j ]->isEmpty= true;
            states [ i ][ j ]->isGoal = false;
            states [ i ][ j ]->isObstacle = false;
            states [ i ][ j ]->isPath = false;
            states [ i ][ j ]->isStart = false;
            states [ i ][ j ]->stateCol = j;
            states [ i ][ j ]->stateHeight = -1;
            states [ i ][ j ]->stateRow = i;
            states [ i ][ j ]->stateWidth = -1;
            states [ i ][ j ]->xInit = -1;
            states [ i ][ j ]->yInit = -1;
            states [ i ][ j ]->isRobot = false;
            states [ i ][ j ]->idRobot = -1;
        }
    }
}

bool Environment::setObstacle(int i, int j)
{
    if ( states [ i ][ j ]->isEmpty )
    {
        states [ i ][ j ]->isEmpty = false;
        states [ i ][ j ]->isObstacle = true;
    }
    return true;
}

bool Environment::clearObstacle(int i, int j)
{
    if ( states [ i ][ j ]->isObstacle )
    {
        states [ i ][ j ]->isEmpty = true;
        states [ i ][ j ]->isObstacle = false;
    }
}

bool Environment::setEmpty(int i, int j)
{
    states [ i ][ j ]->isEmpty = true;

    states [ i ][ j ]->isStart = false;
    states [ i ][ j ]->isGoal = false;
    states [ i ][ j ]->isPath = false;
    states [ i ][ j ]->isObstacle = false;
    states [ i ][ j ]->isRobot = false;
    states [ i ][ j ]->idRobot = -1;

    return true;
}

bool Environment::setGoal(int i, int j)
{
    if ( !states [ i ][ j ]->isEmpty )
        return false;
    states [ i ][ j ]->isGoal = true;
    states [ i ][ j ]->isEmpty = false;
    if ( goal != NULL )
        this->setEmpty ( goal->stateRow, goal->stateCol );
    goal = states [ i ][ j ];
    return true;
}

bool Environment::setStart(int i, int j)
{
    if ( !states [ i ][ j ]->isEmpty )
        return false;
    states [ i ][ j ]->isStart = true;
    states [ i ][ j ]->isEmpty = false;
    if ( start != NULL )
        this->setEmpty ( start->stateRow, start->stateCol );
    start = states [ i ][ j ];
    return true;
}

bool Environment::setNewStatePath(int i, int j)
{
    if ( !states [ i ][ j ]->isEmpty )
        return false;
    states [ i ][ j ]->isPath = false;
    return true;
}

bool Environment::setRobot(int i, int j, int id)
{
    if ( !states [ i ][ j ]->isEmpty )
        return false;
    states [ i ][ j ]->idRobot = id;
    states [ i ][ j ]->isRobot = true;
    states [ i ][ j ]->isEmpty = false;
    return true;
}

void Environment::getStart(int &i, int &j)
{
    i = start->stateRow;
    j = start->stateCol;
}

void Environment::getGoal(int &i, int &j)
{
    i = goal->stateRow;
    j = goal->stateCol;
}

void Environment::clearPath()
{
    for ( int i = 0; i < nRows; i++ )
        for ( int j = 0; j < nCols; j++ )
        {
            states [ i ][ j ]->isPath = false;
//            states [ i ][ j ]->isEmpty = true;
        }
}

void Environment::clearObstacles()
{
    for ( int i = 0; i < nRows; i++ )
        for ( int j = 0; j < nCols; j++ )
            clearObstacle ( i, j );
}

void Environment::restartEnvironment()
{
    clearPath();
    clearObstacles ();
    if ( goal != NULL )
    {
        this->setEmpty ( goal->stateRow, goal->stateCol );
        goal = NULL;
    }
//    this->updateStates( nRows, nCols );
}

void Environment::updateStates(int numberOfLines, int numberOfCols)
{

    for ( int i = 0; i < nRows; i++ )
    {
        for ( int j = 0; j < nCols; j++ )
        {
            delete states [ i ][ j ];
        }
        delete [] states [i];
    }
    delete [] states;

    nRows = numberOfLines;
    nCols = numberOfCols;
    goal = NULL;
    states = new State **[ nRows ];
    for ( int i = 0; i < nRows; i++ )
    {
        states [ i ] = new State *[ nCols ];
        for ( int j = 0; j < nCols; j++ )
        {
            states [ i ][ j ] = new State;
            states [ i ][ j ]->isEmpty= true;
            states [ i ][ j ]->isGoal = false;
            states [ i ][ j ]->isObstacle = false;
            states [ i ][ j ]->isPath = false;
            states [ i ][ j ]->isStart = false;
            states [ i ][ j ]->stateCol = j;
            states [ i ][ j ]->stateHeight = -1;
            states [ i ][ j ]->stateRow = i;
            states [ i ][ j ]->stateWidth = -1;
            states [ i ][ j ]->xInit = -1;
            states [ i ][ j ]->yInit = -1;
            states [ i ][ j ]->isRobot = false;
            states [ i ][ j ]->idRobot = -1;
        }
    }
}
