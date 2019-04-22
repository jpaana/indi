#ifndef EQ500X_H
#define EQ500X_H

class EQ500X : public LX200Generic
{
public:
    class MechanicalPoint
    {
    public:
        MechanicalPoint(double, double);
        MechanicalPoint() {}
    public:
        bool atParkingPosition() const;
    public:
        double RAm() const;
        double DECm() const;
        double RAm(double const);
        double DECm(double const);
    public:
        enum TelescopePierSide setPierSide(enum TelescopePierSide);
    public:
        bool parseStringRA(char const *, size_t);
        bool parseStringDEC(char const *, size_t);
        char const * toStringRA(char *, size_t) const;
        char const * toStringDEC(char *, size_t) const;
    public:
        double RA_degrees_to(MechanicalPoint const &) const;
        double DEC_degrees_to(MechanicalPoint const &) const;
    public:
        double operator -(MechanicalPoint const &) const;
        bool operator !=(MechanicalPoint const &) const;
        bool operator ==(MechanicalPoint const &) const;
    protected:
        enum TelescopePierSide _pierSide {PIER_EAST};
        long _RAm {0*3600}, _DECm {90*3600};
    };
public:
    EQ500X();
    const char *getDefautName();
protected:
    bool getCurrentPosition(MechanicalPoint&);
    bool setTargetPosition(MechanicalPoint const&);
    bool gotoTargetPosition();
protected:
    virtual double getLST();
protected:
    bool isParked();
    void resetSimulation();
protected:
    int sendCmd(char const *);
    int getReply(char *, size_t const);
protected:
    virtual bool checkConnection() override;
    virtual bool updateLocation(double, double, double) override;
    virtual void getBasicData() override;
    virtual bool ReadScopeStatus() override;
    virtual bool initProperties() override;
    virtual bool Goto(double, double) override;
    virtual bool Sync(double, double) override;
    virtual void setPierSide(TelescopePierSide);
private:
    MechanicalPoint currentPosition, targetPosition;
    double previousRA = {0}, previousDEC = {0};
    ln_lnlat_posn lnobserver { 0, 0 };
    int countdown {0};

};

#endif // EQ500X_H
