// Entr�es
#define GPIO_MODE_INPUT_ANALOG        0x0  // Entr�e analogique
#define GPIO_MODE_INPUT_FLOATING      0x4  // Entr�e flottante
#define GPIO_MODE_INPUT_PULL_UP_DOWN  0x8  // Entr�e avec pull-up/pull-down

// Sorties en mode push-pull
#define GPIO_MODE_OUTPUT_PP_10MHz     0x1  // Sortie push-pull 10 MHz
#define GPIO_MODE_OUTPUT_PP_2MHz      0x2  // Sortie push-pull 2 MHz
#define GPIO_MODE_OUTPUT_PP_50MHz     0x3  // Sortie push-pull 50 MHz

// Sorties en mode open-drain
#define GPIO_MODE_OUTPUT_OD_10MHz     0x5  // Sortie open-drain 10 MHz
#define GPIO_MODE_OUTPUT_OD_2MHz      0x6  // Sortie open-drain 2 MHz
#define GPIO_MODE_OUTPUT_OD_50MHz     0x7  // Sortie open-drain 50 MHz

// Sorties en mode fonction alternative push-pull
#define GPIO_MODE_AF_PP_10MHz         0x9  // Fonction alternative push-pull 10 MHz
#define GPIO_MODE_AF_PP_2MHz          0xA  // Fonction alternative push-pull 2 MHz
#define GPIO_MODE_AF_PP_50MHz         0xB  // Fonction alternative push-pull 50 MHz

// Sorties en mode fonction alternative open-drain
#define GPIO_MODE_AF_OD_10MHz         0xD  // Fonction alternative open-drain 10 MHz
#define GPIO_MODE_AF_OD_2MHz          0xE  // Fonction alternative open-drain 2 MHz
#define GPIO_MODE_AF_OD_50MHz         0xF  // Fonction alternative open-drain 50 MHz
void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet) {
    // V�rifier si le num�ro de bit est dans la plage valide (0-15)
    if (num_bit > 15) {return; }// Gestion d'erreur, le num�ro de bit doit �tre entre 0 et 15

    RCC->APB2ENR |= (7 << 2);//pour  GPIOA GPIOB GPIOC
    // D�terminer si le bit est dans CRL ou CRH
    if (num_bit < 8) {
        // CRL pour les bits 0 � 7
        uint32_t mask = 0xF << (num_bit * 4);     // Masque pour effacer les 4 bits correspondants
        uint32_t value = (uint32_t)quartet << (num_bit * 4); // Valeur � d�finir
        GPIOx->CRL = (GPIOx->CRL & ~mask) | value;
    } else {
        // CRH pour les bits 8 � 15
        uint32_t mask = 0xF << ((num_bit - 8) * 4);   // Masque pour effacer les 4 bits correspondants
        uint32_t value = (uint32_t)quartet << ((num_bit - 8) * 4); // Valeur � d�finir
        GPIOx->CRH = (GPIOx->CRH & ~mask) | value;
    }
}
