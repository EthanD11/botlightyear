#ifndef BLY_CAMERATAG_H
#define BLY_CAMERATAG_H


namespace caminibot{

  /**
 * check the orientation of the aruco tag
 * @return 0 if blue; 1 if yellow; 10 if yellow and blue
 */
int tagDetectionOrientation();

/**
 * check the "name" of the aruco tag
 * @param lstTag : list of the code (return)
 *
 * 36 : fragile
 * 13 : resistance
 * 1 to 5 : bleu
 * 6 to 10 : jaune
 * 11 to 50 : aire de jeu pas tout utilisé
 * 51 to 70 : bleu
 * 71 to 90 : jaune
 */
void tagDetectionValue(int* tag);  
}

#endif //MECATROMINIBOT_CAMERATAG_H