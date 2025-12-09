# Data Model: UI Enhancement for Physical AI & Humanoid Robotics Book

## Component Entities

### HeroSection
- **title**: string - Main book title
- **subtitle**: string - Tagline or subtitle
- **description**: string - Additional description
- **primaryButton**: object - Main CTA button
  - text: string
  - link: string
  - type: string (primary/secondary)
- **secondaryButton**: object - Secondary CTA button
  - text: string
  - link: string
  - type: string (primary/secondary)
- **background**: object - Background styling
  - gradient: string (CSS gradient value)
  - pattern: string (optional background pattern)

### FeatureCard
- **title**: string - Card title
- **description**: string - Card description
- **icon**: string - Icon identifier
- **link**: string - Optional link for the card
- **style**: object - Visual styling
  - backgroundColor: string
  - backdropFilter: string
  - border: string

### BookOverview
- **title**: string - Section title
- **content**: string - Overview content
- **layout**: string - Layout type (single-column, two-column, etc.)

### NavbarItem
- **label**: string - Display text
- **href**: string - Link destination
- **position**: string - Position in navbar (left, right)

### FooterSection
- **title**: string - Section title
- **items**: array - List of links/items
  - label: string
  - href: string

## Theme Variables

### Color Palette
- **primary**: #4F46E5 (violet-600)
- **primaryDark**: #4338CA (violet-700)
- **secondary**: #06B6D4 (cyan-500)
- **secondaryDark**: #0891B2 (cyan-600)
- **backgroundLight**: rgba(255, 255, 255, 0.15)
- **backgroundDark**: rgba(255, 255, 255, 0.05)

### Spacing System
- **sectionPadding**: 80px 0
- **cardPadding**: 2.5rem 2rem
- **borderRadius**: 1.25rem
- **shadow**: 0 10px 25px rgba(0, 0, 0, 0.12)

### Typography
- **fontFamily**: Inter, system-ui, sans-serif
- **headingSize**: 3.5rem (main), 2rem (sub)
- **bodySize**: 1rem
- **lineHeight**: 1.6

## State Transitions

### Hover States
- **card**: transform: translateY(-8px), increased shadow
- **button**: transform: translateY(-2px), color change
- **navbarItem**: color change to primary

### Responsive States
- **desktop**: Full layout with all elements visible
- **tablet**: Adjusted spacing and layout
- **mobile**: Stacked layout with adjusted sizes