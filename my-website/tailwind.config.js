// tailwind.config.js
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx,md,mdx}',
    './docs/**/*.{js,jsx,ts,tsx,md,mdx}',
    './blog/**/*.{js,jsx,ts,tsx,md,mdx}',
    './static/**/*.html',
  ],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#f0f9ff',
          100: '#e0f2fe',
          200: '#bae6fd',
          300: '#7dd3fc',
          400: '#38bdf8',
          500: '#00d4ff',  // AI Blue
          600: '#0099ff',   // Darker Blue
          700: '#0369a1',
          800: '#075985',
          900: '#0c4a6e',
        },
        ai: {
          blue: '#00d4ff',
          dark: '#0099ff',
          purple: '#9d4edd',
          darkbg: '#0a0a2a',
          lightbg: '#1a1a40',
        }
      },
      animation: {
        'float': 'float 3s ease-in-out infinite',
        'pulse-glow': 'pulse-glow 2s ease-in-out infinite',
        'gradient': 'gradient 8s linear infinite',
        'shimmer': 'shimmer 2s infinite',
      },
      keyframes: {
        float: {
          '0%, 100%': { transform: 'translateY(0)' },
          '50%': { transform: 'translateY(-10px)' },
        },
        'pulse-glow': {
          '0%, 100%': { opacity: '1' },
          '50%': { opacity: '0.7' },
        },
        gradient: {
          '0%, 100%': {
            'background-size': '200% 200%',
            'background-position': 'left center'
          },
          '50%': {
            'background-size': '200% 200%',
            'background-position': 'right center'
          }
        },
        shimmer: {
          '0%': { backgroundPosition: '-1000px 0' },
          '100%': { backgroundPosition: '1000px 0' },
        }
      },
      backgroundImage: {
        'gradient-ai': 'linear-gradient(135deg, #00d4ff 0%, #0099ff 100%)',
        'gradient-dark': 'linear-gradient(135deg, #0a0a2a 0%, #1a1a40 100%)',
        'gradient-hackathon': 'linear-gradient(135deg, #ff0080 0%, #ff8c00 100%)',
        'grid-pattern': "url(\"data:image/svg+xml,%3Csvg width='60' height='60' viewBox='0 0 60 60' xmlns='http://www.w3.org/2000/svg'%3E%3Cg fill='none' fill-rule='evenodd'%3E%3Cg fill='%2300d4ff' fill-opacity='0.05'%3E%3Cpath d='M36 34v-4h-2v4h-4v2h4v4h2v-4h4v-2h-4zm0-30V0h-2v4h-4v2h4v4h2V6h4V4h-4zM6 34v-4H4v4H0v2h4v4h2v-4h4v-2H6zM6 4V0H4v4H0v2h4v4h2V6h4V4H6z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E\")",
      },
      fontFamily: {
        'sans': ['Inter', 'system-ui', '-apple-system', 'sans-serif'],
        'mono': ['JetBrains Mono', 'monospace'],
      }
    },
  },
  plugins: [],
}