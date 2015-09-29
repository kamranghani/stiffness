package truss;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Kamran
 * Class to store member data.
 */
public class Member {
    //coordinates of near end
    private Joint N;
    //coordinates of far end
    private Joint F;
    //modulus of elasticity
    private double E=1;
    //cross sectional area
    private double A=1;
    //Length 
    private double L=1;

    public double getE() {
        return E;
    }

    public void setE(double E) {
        this.E = E;
    }

    public double getA() {
        return A;
    }

    public void setA(double A) {
        this.A = A;
    }

    public double getL() {
        return L;
    }

    public void setL(double L) {
        this.L = L;
    }
    
    
    public Member(){
        
    }
    public Member(Joint near, Joint far){
        this.N=near;
        this.F=far;
        
    }


    public Joint getN() {
        return N;
    }

    public void setN(Joint N) {
        this.N = N;
    }

    public Joint getF() {
        return F;
    }

    public void setF(Joint F) {
        this.F = F;
    }
    
    public String toString(){
        return(this.getN().getX()+","+this.getN().getY()
                +","+this.getF().getX()+","+this.getF().getY());
         
    }
}
