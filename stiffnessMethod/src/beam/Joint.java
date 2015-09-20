/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package beam;



/**
 *
 * @author Kamran
 */
public class Joint {
    //Coordinate
    private double x;
    private double y;
    
    //Following two variables are used for joint Nobs    
    private int zNo;
    private int yNo;
    
    //following two variables are used for support conditions.
    //Since only two forces (shear in y direction and moment in z direction)
    //are involved in beam therefore we took y & z    
    private boolean zRestrained=false;
    private boolean yRestrained=false;
    
    //forces on joint
    private double fz=0;
    private double fy=0;
    boolean isFzSet=true;   //We will set it true and when we apply support condition, will change it to false
    boolean isFySet=true;
    
    //displacement on joint
    private double dz;
    private double dy;
    
    //following two variables check if the displacement is already given
    private boolean zDisplaced; //For rotation due to moment of the beam
    private boolean yDisplaced; 

    public boolean iszDisplaced() {
        return zDisplaced;
    }

    public void setzDisplaced(boolean zDisplaced) {
        this.zDisplaced = zDisplaced;
    }

    public boolean isyDisplaced() {
        return yDisplaced;
    }

    public void setyDisplaced(boolean yDisplaced) {
        this.yDisplaced = yDisplaced;
    }

    public double getFz() {
        return fz;
    }
    public void setFz(double fz) {
        this.fz = fz;
    }

    public double getFy() {
        return fy;
    }

    public void setFy(double fy) {
        this.fy = fy;
    }

    public double getDz() {
        return dz;
    }

    public boolean isIsFzSet() {
        return isFzSet;
    }

    public void setIsFzSet(boolean isFzSet) {
        this.isFzSet = isFzSet;
    }

    public boolean isIsFySet() {
        return isFySet;
    }

    public void setIsFySet(boolean isFySet) {
        this.isFySet = isFySet;
    }

    
    public void setDz(double dz) {
        this.dz = dz;
    }

    public double getDy() {
        return dy;
    }

    public void setDy(double dy) {
        this.dy = dy;
    }
    

    public boolean iszRestrained() {
        return zRestrained;
    }

    public void setzRestrained(boolean zRestrained) {
        this.zRestrained = zRestrained;
    }

    public boolean isyRestrained() {
        return yRestrained;
    }

    public void setyRestrained(boolean yRestrained) {
        this.yRestrained = yRestrained;
    }
    
    public int getzNo() {
        return zNo;
    }

    public void setzNo(int zNo) {
        this.zNo = zNo;
    }

    public int getyNo() {
        return yNo;
    }

    public void setyNo(int yNo) {
        this.yNo = yNo;
    }
    
    Joint(double x,double y){
        this.x=x;
        this.y=y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
    
    public String toString(){
        return (this.x+","+this.y+" xNo="+this.getzNo()+" yNo="+this.getyNo());
    }
    
}
